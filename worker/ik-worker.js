'use strict';
import { customLogger } from './customLogger.js';
globalThis.__customLogger = customLogger;
const ucl_logger = globalThis.__customLogger;
// ********************************
// デバッグコンソール出力
// if (typeof console.debug === 'function')  ucl_logger.debug = console.debug;
// if (typeof console.log === 'function')  ucl_logger.log = console.log;
if (typeof console.warn === 'function')  ucl_logger.warn = console.warn;
if (typeof console.error === 'function')  ucl_logger.error = console.error;

// Worker script for handling messages and performing calculations
// worker state definition
import {TrapVelocGenerator} from './TrapVelocGenerator.js';
import { encode } from '@msgpack/msgpack';

import { st, sst, bridge } from './workerStateSingleton.js';
import { IkCdCalc } from './ikCdStepClass.js';
// ******** definitions of global variables ********
let makeDoubleVectorG = null; // helper function for DoubleVector

// ****************
// workerの終了フラグ <- 終了時の後始末用
let shutdownFlag = false;

// *************************************
// ******** WASM module loading ********
// *************************************
ucl_logger?.debug('Now intended to import ModuleFactory');

const ModuleFactory = await import('/wasm/slrm_module.js');
ucl_logger?.debug('ModuleFactory: ', ModuleFactory);
ucl_logger?.debug('ModuleFactory.default type:', typeof ModuleFactory.default);
if (typeof ModuleFactory.default !== 'function') {
  ucl_logger?.error('ModuleFactory.default is not a function:', ModuleFactory.default);
  throw new Error('ModuleFactory.default is not a valid function');
}
const SlrmModule = await ModuleFactory.default();
if (!SlrmModule) {
  ucl_logger?.error('Failed to load SlrmModule');
  throw new Error('SlrmModule could not be loaded');
}

SlrmModule.setJsLogLevel(2); // 3: info level, 4: debug level
// CdModule.setJsLogLevel(2); // 3: info level, 4: debug level

// ******** ik and cd calculation object ********
const moveCommandQueue = [];
// const calcObj = new IkCdCalc(SlrmModule, CdModule, moveCommandQueue);
const calcObj = new IkCdCalc(SlrmModule, null, moveCommandQueue);

// ******** worker message handler ********
ucl_logger?.debug('now setting onmessage')
self.onmessage = function(event) {
  const data = event.data;
  let cmdVelGen = calcObj.cmdVelGen;
  switch (data.type) {
  case 'shutdown': // workerを終了する
    if (bridge.socket) {
      bridge.socket.close();
      bridge.socket = null;
    }
    if (SlrmModule) {
      calcObj.deleteSlrm();
      SlrmModule.delete(); // WASMモジュールを解放
    }
    // cd-workerに該当するabを削除するように指示を出す必要がある
    // 現在、cdWorker.jsとlink_shapes_interface.cpp側がサポートしていない
    // if (self.cdWorkerPort) {
    //   self.cdWorkerPort.postMessage({command: 'remove_ab'});
    // }
    self.postMessage({type: 'shutdown_complete'});
    shutdownFlag = true; // workerを終了するフラグを立てる
    break;
  case 'set_worker_loglevel':
    if (data?.logLevel && 0<=data.logLevel && data.logLevel<=4) {
      let logPattern = 0;
      switch (data.logLevel) {
      case 0: break;
      case 1: logPattern = 1; break; // error
      case 2: logPattern = 3; break; // warn
      case 3: logPattern = 7; break; // info
      case 4: logPattern = 15; break; // debug
      default: logPattern = 3; break;
      }
      if (!(logPattern & 1)) { ucl_logger.error = ()=>{}; }
      else { ucl_logger.error = console.error; }
      if (!(logPattern & 2)) { ucl_logger.warn = ()=>{}; }
      else { ucl_logger.warn = console.warn; }
      if (!(logPattern & 4)) { ucl_logger.log = ()=>{}; }
      else { ucl_logger.log = console.log; }
      if (!(logPattern & 8)) { ucl_logger.debug = ()=>{}; }
      else { ucl_logger.debug = console.debug; }
      ucl_logger?.log('Worker log level set to', data.logLevel);
    }
    break;
  case 'set_slrm_loglevel':
    if (data?.logLevel && 0<=data.logLevel && data.logLevel<=4) {
      SlrmModule.setJsLogLevel(data.logLevel);
    }
    break; 
  case 'cd_port': if (data.port) {
    self.cdWorkerPort = data.port;
    self.myBodyName = data.from;
    calcObj.prepareGjkCd(self.cdWorkerPort);
  }
    break;
  case 'init': if (calcObj.state === st.waitingRobotType) {
    calcObj.state = st.generatorMaking;
    ucl_logger?.log('constructing CmdVelGenerator with :', data.filename);
    ucl_logger?.debug('URDF modifier file is', data.modifier);
    // 初期化処理
    const { makeDoubleVector } = createHelpers(SlrmModule);
    makeDoubleVectorG = makeDoubleVector; // グローバルにヘルパー関数を保存
    fetch(data.filename)
      .then(response => response.json())
      .then(jsonData => {
	let urdfIsSorted = false;
	let urdfData = null;
	if (Array.isArray(jsonData)) {
	  urdfData = {...jsonData};
	  urdfIsSorted = true;
	} else {
	  urdfData = jsonData;
	}
	fetch(data.modifier)
	  .then(response => response.json())
	  .then(modifierData => {
	    updateLeaves(urdfData, modifierData);
	    urdfData = Object.values(urdfData);
	    if (!urdfIsSorted) {
	      urdfData = sortJointsByHierarchy(urdfData);
	    }

	    // setting the WASM object's parameters
	    const {jointModelVector,
		   jointModelsArray} = createJointModel(SlrmModule, urdfData);
	    ucl_logger?.debug('type of SlrmModule.CmdVelGen: '
			  + typeof SlrmModule.CmdVelGenerator);
	    cmdVelGen = new SlrmModule.CmdVelGenerator(jointModelVector);
	    cmdVelGen.heapF64 = SlrmModule.HEAPF64;
	    jointModelsArray.forEach(model => model.delete());
	    jointModelVector.delete();
	    if (cmdVelGen === null || cmdVelGen === undefined) {
	      ucl_logger?.error('generation of CmdVelGen instance failed');
	      cmdVelGen = null;
	      return;
	    }
	    if (cmdVelGen !== null && cmdVelGen !== undefined) {
	      ucl_logger?.debug('CmdVelGen instance created:', cmdVelGen);
	    }

	    // prepare the main loop object
	    const revolutes = urdfData.filter(obj =>
	      obj.$.type === 'revolute' ||
	      obj.$.type === 'continuous'
	    );
	    calcObj.prepareVectors(revolutes.length, 16);
	    calcObj.prepareCmdVelGen(cmdVelGen);
	    // joint limitsの設定
	    const jointUpperLimits = [];
	    const jointLowerLimits = [];
	    revolutes.forEach(obj => {
	      jointUpperLimits.push(obj.limit.$.upper);
	      jointLowerLimits.push(obj.limit.$.lower);
	    });
	    calcObj.setJointLimits(jointLowerLimits,
				   jointUpperLimits);
	    ucl_logger?.debug('jointLimits: ', jointUpperLimits,
			  jointLowerLimits);
	    ucl_logger?.debug('Status Definitions: ' +
			  "OK:" + calcObj.SLRM_STAT?.OK + ", " +
			  "ERROR:" + calcObj.SLRM_STAT?.ERROR + ", " +
			  "END:" + calcObj.SLRM_STAT?.END + ", " +
			  "REWIND:" + calcObj.SLRM_STAT?.REWIND + ", " +
			  "SINGULARITY:" + calcObj.SLRM_STAT?.SINGULARITY);
	    cmdVelGen?.setExactSolution(calcObj.exactSolution); // 特異点通過のための設定
	    cmdVelGen?.setLinearVelocityLimit(200.0); // 10 m/s
	    cmdVelGen?.setAngularVelocityLimit(40*Math.PI); // 2Pi rad/s
	    cmdVelGen?.setAngularGain(100.0); // 20 s^-1
	    cmdVelGen?.setLinearGain(100.0); // 20 s^-1
	    const jointVelocityLimit
		  = makeDoubleVector(Array(revolutes.length).fill(Math.PI*2.0)); // 2.0Pi/s // 20Pi rad/s
	    cmdVelGen?.setJointVelocityLimit(jointVelocityLimit); // ジョイント速度制限を設定
	    jointVelocityLimit.delete();

	    if (data.linkShapes && self.cdWorkerPort) {
	      fetch(data.linkShapes)
		.then(response => response.json())
		.then(async linkShapes => {
		  if (linkShapes.length !== revolutes.length + 2) {
		    // +2はbaseとend_effectorの分
		    if (linkShapes.length !== 0)
		      ucl_logger?.error('干渉形状定義の数',
					linkShapes.length,
					'がジョイントの数(+2 effector必須)',
					revolutes.length+2,
					'と一致しません。');
		    return;
		  }
		  // linkShapesは、[[[[x,y,z], ...], ...], ...]のような構造で、最初の[]は全体、
		  // 次の[]は各リンク(rb層でsa層と1対1)、次の[]は各ch層、最後の[]はch層内の頂点(x,y,z)を表す
		  // rb層、sa層、ch層の3段でch層は3xFloat64の配列(vertices)
		  const vertices = []; // あとで3xFloat64 arrayに変換する
		  const rbDataArray = new Int32Array(linkShapes.length);
		  const saOffsets = [];
		  const chOffsets = []; // ch層の開始位置を保存する配列
		  let rbIndex = 0;
		  let chIndex = 0;

		  for (let i = 0; i < linkShapes.length; ++i) {
		    // 当面 linkShapesにはsa層がなくrb層と一対一に対応させる
		    rbDataArray[i] = i;
		    saOffsets.push(rbIndex);
		    rbIndex += linkShapes[i].length;
		    for (let j = 0; j < linkShapes[i].length; ++j) {
		      chOffsets.push(chIndex);
		      chIndex += linkShapes[i][j].length * 3;
		      // ch層の頂点数 * 3 (x,y,z)
		      for (let k = 0; k < linkShapes[i][j].length; ++k) {
			let vertex = linkShapes[i][j][k];
			if (vertex.length !== 3) {
			  ucl_logger?.error('Invalid vertex data at linkShapes['+i+']['+j+']['+k+']: ', vertex);
			  vertex = [0,0,0];
			}
			vertices.push(vertex[0], vertex[1], vertex[2]);
		      }
		    }
		  }
		  // rb層とsa層(index)と頂点データのTypedArrayを作る
		  // rb層とsa層はここでは1対1のためrbOffsetsは0...Nの整数列
		  const saDataArray = new Int32Array(saOffsets);
		  const chDataArray = new Int32Array(chOffsets);
		  const verticesArray = new Float64Array(vertices);
		  const packed = {
		    abLayer: rbDataArray, // abLayer===rbOffsets
		    rbLayer: saDataArray, // rbLayer===saOffsets
		    saLayer: chDataArray, // saLayer===chOffsets
		    vertices: verticesArray, // 頂点データのTypedArray
		  };

		  // fetch test pairs from data.testPairs if exists
		  let testPairs = [];
		  if (!data.testPairs) {
		    for (let i=0; i< linkShapes.length-4; i++) {
		      for (let j=i+2; j<linkShapes.length; j++) {
			testPairs.push([i,j]);
		      }
		    }
		    ucl_logger?.debug('using default test pairs: ',
				      testPairs);
		  } else {
		    ucl_logger?.debug('fetch test pairs from',
				      data.testPairs);
		    const response = await fetch(data.testPairs);
		    testPairs = await response.json();
		  }
		  packed.testPairs = testPairs;
		  // cdWorkerにtransferable objectとして送る
		  self.cdWorkerPort.postMessage({command: 'link_shapes',
						 shapes: packed,
						 name: self.myBodyName},
						[packed.abLayer.buffer,
						 packed.rbLayer.buffer,
						 packed.saLayer.buffer,
						 packed.vertices.buffer]);
		})
		.catch(error => {
		  ucl_logger?.error('Error fetching or parsing SHAPE file:',
				    error);
		});
	    }
	    if (data.bridgeUrl) {
	      ucl_logger?.debug('recieve bridge URL: ', data.bridgeUrl);
	      // bridge用のURLが付いているためbridgeが使える
	      bridge.url = data.bridgeUrl;
	      bridge.connect();
	    }
	    // なにかの加減でオブジェクト生成に失敗した場合はここでエラーがthrownされる
	    calcObj.state = st.generatorReady;
	    self.postMessage({type: 'generator_ready'});
	  })
	  .catch(error => {
	    ucl_logger?.warn('Error fetching or parsing URDF modifier file:', error);
	    ucl_logger?.warn('modifier file name:', data.modifier);
	  });
      })
      .catch(error => {
	ucl_logger?.error('Error fetching or parsing URDF.JSON file:', error);
      });
  } break;
  case 'set_initial_joints': if (calcObj.state === st.generatorReady ||
				 calcObj.state === st.slrmReady) {
    if (data.joints) {
      // 初期ジョイントの設定処理
      const joints = new Float64Array(data.joints.length);
      joints.set(data.joints);
      calcObj.joints = joints;
      // prepareRewindQueueu()の前にcalcObj.jointsのセットが必要
      calcObj.prepareRewindQueue();
      const initialJoints = joints.slice();
      calcObj.initialjoints = initialJoints;
      calcObj.prevJoints = joints.slice();
      // velocities = new Float64Array(joints.length);
      ucl_logger?.debug('Setting initial joints:'
		  +joints.map(v => (v*57.2958).toFixed(1)).join(', '));
      if (!calcObj.jointRewinder ||
	  joints.length !== calcObj.jointRewinder.length) {
	// 面倒なので、ジョイント数が変わった場合はjointRewinderを全部再生成
	// jointRewinder = Array.from({length: joints.length}, ()=>new TrapVelocGenerator(5,1,1,0.0625));
	calcObj.jointRewinder = Array(joints.length).fill(null)
	  .map((_, i) => {
	    if (i<=1) { // joint 1, 2は特に遅くする
	      return new TrapVelocGenerator(5, 1, 0.2, 0.02);
	    } else {
	      return new TrapVelocGenerator(5, 1, 1, 0.0625); // 5s, 1m/s, 1rad/s, 0.0625s
	    }
	  });
      }
      calcObj.jointRewinder
	.forEach((der,ix)=>{der.reset(); der.setX0(initialJoints[ix])});
      calcObj.state = st.slrmReady;
      calcObj.noDestination = true;
      calcObj.subState = sst.moving; // 目標位置に移動中
      // calcObj.subState = sst.converged;
      ucl_logger?.log('Worker state changed to slrmReady');
    }
  } break;
  case 'destination': if (calcObj.state === st.slrmReady &&
			  calcObj.subState !== sst.rewinding &&
			  calcObj.subState !== sst.jMoving &&
			  data.endLinkPose ) {
    // データの受信処理
    //newDestinationFlag = true; // 新しいdestinationが来た
    calcObj.controllerTfVec.set(data.endLinkPose);
    ucl_logger?.debug('Received destination: '
		+ calcObj.controllerTfVec[12].toFixed(3) + ', '
		+ calcObj.controllerTfVec[13].toFixed(3) + ', '
		+ calcObj.controllerTfVec[14].toFixed(3));
    calcObj.subState = sst.moving;
  } break;
  case 'set_joint_targets':
    if (data.jointTargets &&
	calcObj.state === st.slrmReady ) {
      if (calcObj.subState !== sst.rewinding &&
	  calcObj.subState !== sst.moving ) {
	if (data.jointTargets.length === calcObj.joints.length) {
	  // controllerJointVec = [...data.jointTargets];
	  calcObj.controllerJointVec.set(data.jointTargets);
	  calcObj.subState = sst.jMoving;
	} else {
	  ucl_logger?.error('set_joint_targets: jointTargets length mismatch:',
					   data.jointTargets.length, 'vs',
					   calcObj.joints.length);
	}
      } else {
	const cmd = { type: 'jMove', joints: data.jointTargets };
	moveCommandQueue.push(cmd);
      }
    } else {
      ucl_logger?.warn('Ignored set_joint_targets command.');
      ucl_logger?.warn('set_joint_targets: invalid state or missing jointTargets');
      ucl_logger?.warn('  calcObj.state:', calcObj.state, ' calcObj.subState:', calcObj.subState);
    }
    break;
  case 'slow_rewind':
    if (calcObj.state === st.slrmReady && calcObj.jointRewinder) {
      if (data.slowRewind == true) {
	calcObj.subState = sst.rewinding;
      } else {
	calcObj.subState = sst.converged;
      }
    }
    break;
  case 'set_end_effector_point':
  case 'set_end_effector_position':
  case 'set_end_effector_orientation':
  case 'set_end_effector_pose':
    // calcObj.stateとcalcObj.subStateが何のときに可能とするかは未定
    if (makeDoubleVectorG) {

      if (data.endEffectorPoint &&
	  data.endEffectorPoint.length === 3 &&
	  typeof data.endEffectorPoint[0] === 'number' &&
	  typeof data.endEffectorPoint[1] === 'number' &&
	  typeof data.endEffectorPoint[2] === 'number') {
	const endEffectorPosition = makeDoubleVectorG(data.endEffectorPoint);
	cmdVelGen?.setEndEffectorPosition(endEffectorPosition);
	endEffectorPosition.delete();
      }
      if (data.endEffectorQuaternion &&
	  data.endEffectorQuaternion.length === 4 &&
	  typeof data.endEffectorQuaternion[0] === 'number' &&
	  typeof data.endEffectorQuaternion[1] === 'number' &&
	  typeof data.endEffectorQuaternion[2] === 'number' &&
	  typeof data.endEffectorQuaternion[3] === 'number') {
	const eigenQuat = [ data.endEffectorQuaternion[3],
			    data.endEffectorQuaternion[0],
			    data.endEffectorQuaternion[1],
			    data.endEffectorQuaternion[2] ];
	const endEffectorOrientation = makeDoubleVectorG(eigenQuat);
	cmdVelGen?.setEndEffectorOrientation(endEffectorOrientation);
	endEffectorOrientation.delete();
      }
      const tmp = calcObj.subState;
      calcObj.subState = sst.moving; // アームをee移動分だけ動かすために一回呼ぶ
      // endLinkPoseVec = []; // 現在値をゴールにしてcalcVelocityPQを1回実行する
      calcObj.noDestination = true;
      // mainFunc(0); // ここでeeの位置を更新
      calcObj.step(0);
      calcObj.subState = tmp; // 元の状態に戻す
    }
    break;
  case 'set_exact_solution':
    if (calcObj.state === st.generatorReady || calcObj.state === st.slrmReady) {
      if (data.exactSolution !== undefined) {
	if (data.exactSolution === true) {
	  calcObj.exactSolution = true;
	} else {
	  calcObj.exactSolution = false;
	}
	cmdVelGen?.setExactSolution(calcObj.exactSolution);
	ucl_logger?.log('Exact solution for singularity set to: ',
		    calcObj.exactSolution);
      }
    }
    break;
  case 'set_joint_weights':
    if (calcObj.state === st.generatorReady ||
	calcObj.state === st.slrmReady) {
      if (data.jointNumber !== undefined &&
	  data.jointWeight !== undefined) {
	if (cmdVelGen?.setJointWeight &&
	    cmdVelGen?.setJointWeight(data.jointNumber, data.jointWeight) !== true) {
	  ucl_logger?.error('set_joint_weights: failed to set weight for joint number ',
			data.jointNumber);
	}
      }
    }
    break;
  case 'set_joint_desirable_vlimit':
    if (calcObj.state === st.generatorReady ||
	calcObj.state === st.slrmReady) {
      if (data.jointNumber === undefined) { data.jointNumber = -1; } // 全関節に適用
      if (data.velocityLimit !== undefined) {
	if (cmdVelGen?.setJointDesirableVelocityLimit &&
	    cmdVelGen?.setJointDesirableVelocityLimit(data.jointNumber,
						     data.velocityLimit) !== true) {
	  ucl_logger?.error('set_joint_desirable_vlimit: failed to set desirable velocity limit for joint number ',
			data.jointNumber);
	}
      }
    }
    break;
  case 'clear_joint_desirable':
    if (calcObj.state === st.generatorReady ||
	calcObj.state === st.slrmReady) {
      if (data.jointNumber !== undefined) {
	if (cmdVelGen?.setJointDesirable &&
	    cmdVelGen?.setJointDesirable(data.jointNumber, false) !== true) {
	  ucl_logger?.error('clear_joint_desirable: failed to clear desirable for joint number ',
			data.jointNumber);
	}
      }
    }
    break;
  case 'set_joint_desirable':
    ucl_logger?.debug('in worker, set_joint_desirable called:', data);
    if (calcObj.state === st.generatorReady || calcObj.state === st.slrmReady) {
      if (data.jointNumber !== undefined &&
	  data.lower !== undefined && data.upper !== undefined &&
	  data.gain !== undefined) {
	ucl_logger?.debug('in worker, set_joint_desirable: jointNumber=', data.jointNumber,
		    ' lower=', data.lower,
		    ' upper=', data.upper,
		    ' gain=', data.gain);
	if (cmdVelGen?.setJointDesirable &&
	    cmdVelGen?.setJointDesirable(data.jointNumber, true,
					data.lower,
					data.upper,
					data.gain) !== true) {
	  ucl_logger?.error('set_joint_desirable: failed to set desirable for joint number ',
			data.jointNumber);
	}
      }
    }
    break;
  // case 'set_linear_velocity_limit':
  // case 'set_angular_velocity_limit':
  // case 'set_linear_gain':
  // case 'set_angular_gain':
  case 'set_joint_velocity_limit':
    if (calcObj.state === st.generatorReady || calcObj.state === st.slrmReady) {
      if (data.velocityLimit !== undefined) {
	const jointVelocityLimit
	      = makeDoubleVectorG(data.velocityLimit);
	if (cmdVelGen?.setJointVelocityLimitSingle(jointVelocityLimit) !== true) {
	  ucl_logger?.error('set_joint_velocity_limit: failed to set joint velocity limit');
	}
	jointVelocityLimit.delete();
      } else {
	ucl_logger?.error('set_joint_velocity_limit: velocityLimit is undefined');
      }
    }
    break;
  case 'set_ignore_collisions':
    if (calcObj.state === st.generatorReady ||
	calcObj.state === st.slrmReady) {
      if (data.ignoreCollisions !== undefined) {
	calcObj.ignoreCollision = data.ignoreCollisions;
	ucl_logger?.log('Ignore collisions set to: ',
		    calcObj.ignoreCollision);
      }
    }
    break;
  case 'set_ignore_joint_limits':
    if (calcObj.state === st.generatorReady ||
	calcObj.state === st.slrmReady) {
      if (data.ignoreJointLimits !== undefined) {
	calcObj.ignoreJointLimits = data.ignoreJointLimits;
	ucl_logger?.log('Ignore joint limits set to: ', calcObj.ignoreJointLimits);
      }
    }
    break;

  case 'set_joint_limit_keep_moving':
    ucl_logger?.debug('Joint limit keep moving: command received');
    ucl_logger?.debug('data:', data);
    ucl_logger?.debug('arg:', data.jointLimitKeepMoving);
    ucl_logger?.debug('calcObj.state:', calcObj.state);
    if (calcObj.state === st.generatorReady ||
	calcObj.state === st.slrmReady) {
      if (data.jointLimitKeepMoving !== undefined) {

	calcObj.jointLimitKeepMoving = data.jointLimitKeepMoving;
	ucl_logger?.log('Joint limit keep moving set to: ',
			calcObj.jointLimitKeepMoving);
      }
    }
    break;
  default:
    break;
  }
};


// ******** main loop ********
// const timeInterval = 4; // time step for simulation in milliseconds
// const logInterval = 0n/BigInt(timeInterval); // log interval in BigInt
// // ******** control parameters ********
// let counter = 0n;
// let jMoveGain = 10.0; // joint move command gain
// let jMoveVelocityLimit = Math.PI/3.0; // joint move command velocity limit rad/s



// ******** worker main loop ********
function mainLoop(prevTime = performance.now()-calcObj.timeInterval) {
  const now = performance.now();
  const deltaTime = now - prevTime;
  // ここにstepの上限をつけること。stepが長くなりすぎると速度フィードバックが破綻する
  calcObj.step(deltaTime / 1000); // time step in seconds
  if (shutdownFlag === true) {
    self.postMessage({type: 'shutdown_complete'});
    ucl_logger?.log('main loop was finished')
    self.close()
    return
  }
  if (bridge.socket) {
    const end = performance.now();
    const duration = end - now;
    const startSec = Math.floor(duration / 1000);
    const startNanosec = Math.floor((duration - startSec * 1000) * 1e6);
    const msg = {
      topic: 'timeRef',
      javascriptStamp: Date.now(),
      header: {frame_id: 'none'},
      time_ref: { sec: startSec,
		  nanosec: startNanosec },
      source: 'slrm_and_cd'
    };
    const binary = encode(msg);
    if (bridge.socket.readyState === WebSocket.OPEN) {
      bridge.socket.send(binary);
    }
  }
  setTimeout(() => mainLoop(now), 0); // 次のループをスケジュール
}

// ******** worker start ********
calcObj.state = st.waitingRobotType;
self.postMessage({type: 'ready'});
mainLoop(); // メインループを開始
// event loop



// ******** THE FOLLOWING FUNCTIONS ARE HOISTED ********
// ******** helper functions ********
// SlrmModuleを閉じ込めて、その関連オブジェクトを生成するhelper関数群
function createHelpers(module) {
  function makeDoubleVector(jsArray) {
    const vec = new module.DoubleVector();
    for (let i = 0; i < jsArray.length; ++i) {
      vec.push_back(jsArray[i]);
    }
    return vec;
  }
  // 他のヘルパー関数もここに追加できる
  return {
    makeDoubleVector,
    // ... more helpers
  };
}

// ******** wasmObj constructor's arg generator ********
// CmdVelGeneratorのconstructorの引数生成
function createJointModel(mod, list) {
  // 各行をJointModelFlatStructに変換
  function modDoubleVector(mod, jsArray) {
    const vec = new mod.DoubleVector();
    for (let i = 0; i < jsArray.length; ++i) {
      vec.push_back(jsArray[i]);
    }
    return vec;
  }
  function modJointModelVector(mod, jmArray) {
    const vec = new mod.JointModelFlatStructVector();
    for (let i = 0; i < jmArray.length; ++i) {
      vec.push_back(jmArray[i]);
    }
    return vec;
  }
  const jointTypeFromString = {
    revolute: mod.JointType.Revolute,
    continuous: mod.JointType.Continuous,
    prismatic: mod.JointType.Prismatic,
    fixed: mod.JointType.Fixed,
    floating: mod.JointType.Floating,
    planar: mod.JointType.Planar,
  };

  const jointModelsArray = list.map(obj => {
    const xyz_in = obj.origin.$.xyz ?? [0,0,0];// [NaN, NaN, NaN];
    const xyz = modDoubleVector(mod,
				Array.isArray(xyz_in) && xyz_in.length === 3
				? xyz_in : [0,0,0]);//[NaN, NaN, NaN]);
    const rpy_in = obj.origin.$.rpy ?? [0,0,0];//[NaN, NaN, NaN];
    const rpy = modDoubleVector(mod,
				Array.isArray(rpy_in) && rpy_in.length === 3
				? rpy_in : [0,0,0]);//[NaN, NaN, NaN]);
    const axis_in = obj.axis?.$?.xyz ?? [0,0,1];//[NaN, NaN, NaN];
    const axis = modDoubleVector(mod,
				 Array.isArray(axis_in) && axis_in.length === 3
				 ? axis_in : [0,0,1]);//[NaN, NaN, NaN]);
    let jt = jointTypeFromString[obj.$?.type];
    if (!jt) {
      ucl_logger?.error('Unknown joint type string:', obj.$?.type,
		    'setting to fixed');
      jt = jointTypeFromString.fixed;
    }
    const jointModel = new mod.JointModelFlatStruct(axis, xyz, rpy, jt);
    axis.delete();
    xyz.delete();
    rpy.delete();
    return jointModel;
  });
  const jointModelVector = modJointModelVector(mod,jointModelsArray);
  return { jointModelVector, jointModelsArray }
}

// ******** utility functions for JSON parsing ********
//
function sortJointsByHierarchy(urdfData) {
  const graph = new Map(); // parent -> list of joints
  const inDegree = new Map(); // child link name -> number of parents
  const linkToJoint = new Map(); // child link -> joint object (for ordered result)
  urdfData.forEach(joint => {
    const parent = joint.parent.$.link;
    const child = joint.child.$.link;
    if (!graph.has(parent)) { graph.set(parent, []); }
    graph.get(parent).push(joint);
    inDegree.set(child, (inDegree.get(child) || 0) + 1);
    if (!inDegree.has(parent)) { inDegree.set(parent, 0); }
    linkToJoint.set(child, joint);
  });
  const queue = [];
  for (const [link, degree] of inDegree.entries()) {
    if (degree === 0) { queue.push(link); }
  }
  const orderedJoints = [];
  while (queue.length > 0) {
    const parentLink = queue.shift();
    const children = graph.get(parentLink) || [];
    for (const joint of children) {
      const childLink = joint.child.$.link;
      orderedJoints.push(joint);
      inDegree.set(childLink, inDegree.get(childLink) - 1);
      if (inDegree.get(childLink) === 0) {
	queue.push(childLink);
      }
    }
  }
  if (orderedJoints.length !== urdfData.length) {
    ucl_logger?.warn('Cycle detected or disconnected components in URDF joints');
  }
  return orderedJoints;
}

function updateLeaves(a, b) {
  for (const key in b) {
    if (!(key in a)) {
      ucl_logger?.debug('key in update.json:',key,' ignored');
      continue; // aに存在しないキーは無視
    }
    const bVal = b[key];
    const aVal = a[key];
    if (
      bVal !== null &&
      typeof bVal === "object" &&
      !Array.isArray(bVal) &&
      aVal !== null &&
      typeof aVal === "object" &&
      !Array.isArray(aVal)
    ) {
      updateLeaves(aVal, bVal); // 両方オブジェクトなら再帰
    } else {
      ucl_logger?.warn('key:',key,'val:',a[key],'is replaced by',bVal);
      a[key] = bVal; // 配列やオブジェクトでない値は上書き
    }
  }
  return a;
}
