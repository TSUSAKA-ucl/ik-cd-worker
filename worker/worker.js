'use strict';
// Worker script for handling messages and performing calculations
// worker state definition
import {TrapVelocGenerator} from './TrapVelocGenerator.js';
import { encode } from '@msgpack/msgpack';

import { st, sst, bridge } from './workerStateSingleton.js';
import { IkCdCalc } from './ikCdStepClass.js';
// ******** definitions of global variables ********
// let mainLoopObj = null; // main loop object
// let cmdVelGen = null; // コマンド速度生成器WASMオブジェクト
// let gjkCd = null; // collision detection WASMオブジェクト
let makeDoubleVectorG = null; // helper function for DoubleVector
// let makeCdDoubleVectorG = null;

// ******** flags ********
// mainLoopObjのプロパティーとしてアクセスする
// let exactSolution = false; // singularity通過のための設定
// let ignoreCollision = false; // 干渉判定を無視するかどうか
// let ignoreJointLimits = false; // ジョイントリミットを無視するかどうか
// ******** vector variables ********
// 将来的にはmainLoopObjのプロパティーとしてアクセスする
// let endLinkPoseVec = []; // endLinkPoseの値を受け取るベクトル
// let controllerTfVec = null; // controllerから受け取った目標位置姿勢ベクトル
// let controllerJointVec = null; // controllerから受け取った目標ジョイントベクトル
// let initialJoints = null;
// let jointRewinder = null;
// let joints = null; // joint position vector. size is 6,7 or 8
// let prevJoints = null; // 前回のジョイントポジション
// let velocities = null;
// let logPrevJoints = null; // ログ出力用の前回ジョイントポジション
// const jointUpperLimits = [];
// const jointLowerLimits = [];

// ****************
// workerの終了フラグ <- 終了時の後始末用
let shutdownFlag = false;
// ********************************

// *************************************
// ******** WASM module loading ********
// *************************************
console.debug('Now intended to import ModuleFactory');
// import ModuleFactory from '/wasm/slrm_module.js';
const ModuleFactory = await import('/wasm/slrm_module.js');
const CdModuleFactory = await import('/wasm/cd_module.js');
console.debug('ModuleFactory: ', ModuleFactory);
console.debug('ModuleFactory.default type:', typeof ModuleFactory.default);
if (typeof ModuleFactory.default !== 'function') {
  console.error('ModuleFactory.default is not a function:', ModuleFactory.default);
  throw new Error('ModuleFactory.default is not a valid function');
}
const SlrmModule = await ModuleFactory.default();
if (!SlrmModule) {
  console.error('Failed to load SlrmModule');
  throw new Error('SlrmModule could not be loaded');
}
const CdModule = await CdModuleFactory.default();
if (!CdModule) {
  console.error('Failed to load CdModule');
  throw new Error('CdModule could not be loaded');
}

SlrmModule.setJsLogLevel(2); // 3: info level, 4: debug level
CdModule.setJsLogLevel(2); // 3: info level, 4: debug level

// ******** ik and cd calculation object ********
const calcObj = new IkCdCalc(SlrmModule, CdModule);

// ******** worker message handler ********
console.debug('now setting onmessage')
self.onmessage = function(event) {
  const data = event.data;
  let cmdVelGen = calcObj.cmdVelGen;
  let gjkCd = calcObj.gjkCd;
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
    if (CdModule) {
      calcObj.deleteCd();
      CdModule.delete(); // WASMモジュールを解放
    }
    self.postMessage({type: 'shutdown_complete'});
    shutdownFlag = true; // workerを終了するフラグを立てる
    break;
  case 'set_slrm_loglevel':
    if (data?.logLevel && 0<=data.logLevel && data.logLevel<=4) {
      SlrmModule.setJsLogLevel(data.logLevel);
    }
    break; 
  case 'set_cd_loglevel':
    if (data?.logLevel && 0<=data.logLevel && data.logLevel<=4) {
      CdModule.setJsLogLevel(data.logLevel);
    }
    break; 
  case 'init': if (calcObj.state === st.waitingRobotType) {
    calcObj.state = st.generatorMaking;
    console.log('constructing CmdVelGenerator with :', data.filename);
    console.debug('URDF modifier file is', data.modifier);
    // 初期化処理
    const { makeDoubleVector } = createHelpers(SlrmModule);
    const { makeCdDoubleVector, makeConvexShape } = createCdHelpers(CdModule);
    makeDoubleVectorG = makeDoubleVector; // グローバルにヘルパー関数を保存
    // makeCdDoubleVectorG = makeCdDoubleVector; // グローバルにヘルパー関数を保存
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
	    const revolutes = urdfData.filter(obj => obj.$.type === 'revolute');
	    calcObj.prepareVectors(revolutes.length, 16);

	    const {jointModelVector,
		   jointModelsArray} = createJointModel(SlrmModule, revolutes);
	    console.debug('type of SlrmModule.CmdVelGen: '
			  + typeof SlrmModule.CmdVelGenerator);
	    cmdVelGen = new SlrmModule.CmdVelGenerator(jointModelVector);
	    // console.debug("type of jointModels is ", typeof jointModels);
	    jointModelsArray.forEach(model => model.delete());
	    jointModelVector.delete();
	    if (cmdVelGen === null || cmdVelGen === undefined) {
	      console.error('generation of CmdVelGen instance failed');
	      cmdVelGen = null;
	      return;
	    }
	    if (cmdVelGen !== null && cmdVelGen !== undefined) {
	      console.debug('CmdVelGen instance created:', cmdVelGen);
	    }
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
	    console.debug('jointLimits: ', jointUpperLimits,
			  jointLowerLimits);
	    console.debug('Status Definitions: ' +
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

	    if (data.linkShapes) {
	      const {jointModelVector,
		     jointModelsArray} = createJointModel(CdModule, revolutes);
	      const basePosition = makeCdDoubleVector([0.0, 0.0, 0.0]);
	      const baseOrientation = makeCdDoubleVector([1.0, 0.0, 0.0, 0.0]);
	      gjkCd = new CdModule.CollisionDetection(jointModelVector,
						      basePosition,
						      baseOrientation);
	      // jointModels.forEach(model => model.delete());
	      jointModelVector.delete();
	      jointModelsArray.forEach(model => model.delete());
	      basePosition.delete();
	      baseOrientation.delete();
	    }
	    if (gjkCd) {
	      fetch(data.linkShapes)
		.then(response => response.json())
		.then(async linkShapes => {
		  if (linkShapes.length !== revolutes.length + 2) { // +2はbaseとend_effectorの分
		    if (linkShapes.length !== 0)
		      console.error('干渉形状定義の数', linkShapes.length,
				    'がジョイントの数(+2 effector必須)', revolutes.length+2,
				    'と一致しません。');
		    return;
		  }
		  console.log('linkShapes.length in',data.linkShapes,': ', linkShapes.length);
		  for (let i = 0; i < linkShapes.length; ++i) {
		    // console.debug(`リンク番号${i} のvector生成`);
		    const shapeWasm = new CdModule.ConvexShapeVector();
		    for (const convex of linkShapes[i]) {
		      const convexWasm = makeConvexShape(convex);
		      // console.debug('size of convex js: ', convex.length);
		      shapeWasm.push_back(convexWasm);
		      convexWasm.delete();
		    }
		    gjkCd.addLinkShape(i, shapeWasm);
		    shapeWasm.delete();
		  }
		  console.debug('setting up of link shapes is finished');
		  gjkCd.infoLinkShapes();
		  // fetch test pairs from data.testPairs if exists
		  if (!data.testPairs) {
		    // const testPairs = [[0,2],[0,3],[0,4],[0,5],[0,6],[0,7],
		    // 		       [1,3],[1,4],[1,5],[1,6],[1,7],
		    // 		       [2,4],[2,5],[2,6],[2,7],
		    // 		       [3,5],[3,6],[3,7]
		    // 		      ];
		    const testPairs = [];
		    for (let i=0; i< linkShapes.length-4; i++) {
		      for (let j=i+2; j<linkShapes.length; j++) {
			testPairs.push([i,j]);
		      }
		    }
		    console.debug('using default test pairs: ', testPairs);
		    gjkCd.clearTestPairs();
		    for (const pair of testPairs) {
		      gjkCd.addTestPair(pair[0],pair[1]);
		    }
		  } else {
		    console.debug('recieve test pairs from', data.testPairs);
		    const response = await fetch(data.testPairs);
		    const testPairs = await response.json();
		    gjkCd.clearTestPairs();
		    for (const pair of testPairs) {
		      gjkCd.addTestPair(pair[0],pair[1]);
		    }
		  }
		  return gjkCd;
		})
		.then((gjkCdInstance) => {
		  calcObj.prepareGjkCd(gjkCdInstance);
		})
		.catch(error => {
		  console.error('Error fetching or parsing SHAPE file:', error);
		});
	    }
	    if (data.bridgeUrl) {
	      console.debug('recieve bridge URL: ', data.bridgeUrl);
	      // bridge用のURLが付いているためbridgeが使える
	      bridge.url = data.bridgeUrl;
	      bridge.connect();
	    }
	    // なにかの加減でオブジェクト生成に失敗した場合はここでエラーがthrownされる
	    calcObj.state = st.generatorReady;
	    self.postMessage({type: 'generator_ready'});
	  })
	  .catch(error => {
	    console.warn('Error fetching or parsing URDF modifier file:', error);
	    console.warn('modifier file name:', data.modifier);
	  });
      })
      .catch(error => {
	console.error('Error fetching or parsing URDF.JSON file:', error);
      });
  } break;
  case 'set_initial_joints': if (calcObj.state === st.generatorReady ||
				 calcObj.state === st.slrmReady) {
    if (data.joints) {
      // 初期ジョイントの設定処理
      const joints = new Float64Array(data.joints.length);
      joints.set(data.joints);
      calcObj.joints = joints;
      const initialJoints = joints.slice();
      calcObj.initialjoints = initialJoints;
      calcObj.prevJoints = joints.slice();
      // velocities = new Float64Array(joints.length);
      console.debug('Setting initial joints:'
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
      console.log('Worker state changed to slrmReady');
    }
  } break;
  case 'destination': if (calcObj.state === st.slrmReady &&
			  calcObj.subState !== sst.rewinding &&
			  calcObj.subState !== sst.jMoving &&
			  data.endLinkPose ) {
    // データの受信処理
    //newDestinationFlag = true; // 新しいdestinationが来た
    calcObj.controllerTfVec.set(data.endLinkPose);
    console.debug('Received destination: '
		+ calcObj.controllerTfVec[12].toFixed(3) + ', '
		+ calcObj.controllerTfVec[13].toFixed(3) + ', '
		+ calcObj.controllerTfVec[14].toFixed(3));
    calcObj.subState = sst.moving;
  } break;
  case 'set_joint_targets':
    if (data.jointTargets &&
	calcObj.state === st.slrmReady &&
	calcObj.subState !== sst.rewinding &&
	calcObj.subState !== sst.moving ) {
      if (data.jointTargets.length === calcObj.joints.length) {
	// controllerJointVec = [...data.jointTargets];
	calcObj.controllerJointVec.set(data.jointTargets);
	calcObj.subState = sst.jMoving;
      } else {
	console.error('set_joint_targets: jointTargets length mismatch:',
		      data.jointTargets.length, 'vs',
		      calcObj.joints.length);
      }
    } else {
      console.warn('Ignored set_joint_targets command.');
      console.warn('set_joint_targets: invalid state or missing jointTargets');
      console.warn('  calcObj.state:', calcObj.state, ' calcObj.subState:', calcObj.subState);
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
    // calcObj.stateとcalcObj.subStateが何のときに可能とするかは未定
    if (data.endEffectorPoint && makeDoubleVectorG) {
      // if (data.endEffectorPoint === null) {
      // 	const ee = cmdVelGen?.getEndEffectorPosition();
      // 	const endEffectorPoint = [ee.get(0), ee.get(1), ee.get(2)];
      // 	ee.delete();
      // 	self.postMessage({type: 'end_effector_point',
      // 			  endEffectorPoint: endEffectorPoint});
      // }
      if (data.endEffectorPoint.length === 3 &&
	  typeof data.endEffectorPoint[0] === 'number' &&
	  typeof data.endEffectorPoint[1] === 'number' &&
	  typeof data.endEffectorPoint[2] === 'number') {
	console.debug('Setting end effector point: ', data.endEffectorPoint);
	const endEffectorPosition = makeDoubleVectorG(data.endEffectorPoint);
	cmdVelGen?.setEndEffectorPosition(endEffectorPosition);
	endEffectorPosition.delete();
	const tmp = calcObj.subState;
	calcObj.subState = sst.moving; // アームをee移動分だけ動かすために一回呼ぶ
	// endLinkPoseVec = []; // 現在値をゴールにしてcalcVelocityPQを1回実行する
	calcObj.noDestination = true;
	// mainFunc(0); // ここでeeの位置を更新
	calcObj.step(0);
	calcObj.subState = tmp; // 元の状態に戻す
      }
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
	console.log('Exact solution for singularity set to: ',
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
	  console.error('set_joint_weights: failed to set weight for joint number ',
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
	  console.error('set_joint_desirable_vlimit: failed to set desirable velocity limit for joint number ',
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
	  console.error('clear_joint_desirable: failed to clear desirable for joint number ',
			data.jointNumber);
	}
      }
    }
    break;
  case 'set_joint_desirable':
    console.debug('in worker, set_joint_desirable called:', data);
    if (calcObj.state === st.generatorReady || calcObj.state === st.slrmReady) {
      if (data.jointNumber !== undefined &&
	  data.lower !== undefined && data.upper !== undefined &&
	  data.gain !== undefined) {
	console.debug('in worker, set_joint_desirable: jointNumber=', data.jointNumber,
		    ' lower=', data.lower,
		    ' upper=', data.upper,
		    ' gain=', data.gain);
	if (cmdVelGen?.setJointDesirable &&
	    cmdVelGen?.setJointDesirable(data.jointNumber, true,
					data.lower,
					data.upper,
					data.gain) !== true) {
	  console.error('set_joint_desirable: failed to set desirable for joint number ',
			data.jointNumber);
	}
      }
    }
    break;
  case 'set_joint_velocity_limit':
    if (calcObj.state === st.generatorReady || calcObj.state === st.slrmReady) {
      if (data.velocityLimit !== undefined) {
	const jointVelocityLimit
	      = makeDoubleVectorG(data.velocityLimit);
	if (cmdVelGen?.setJointVelocityLimitSingle(jointVelocityLimit) !== true) {
	  console.error('set_joint_velocity_limit: failed to set joint velocity limit');
	}
	jointVelocityLimit.delete();
      } else {
	console.error('set_joint_velocity_limit: velocityLimit is undefined');
      }
    }
    break;
  case 'set_ignore_collisions':
    if (calcObj.state === st.generatorReady ||
	calcObj.state === st.slrmReady) {
      if (data.ignoreCollisions !== undefined) {
	calcObj.ignoreCollision = data.ignoreCollisions;
	console.log('Ignore collisions set to: ',
		    calcObj.ignoreCollision);
      }
    }
    break;
  case 'set_ignore_joint_limits':
    if (calcObj.state === st.generatorReady ||
	calcObj.state === st.slrmReady) {
      if (data.ignoreJointLimits !== undefined) {
	calcObj.ignoreJointLimits = data.ignoreJointLimits;
	console.log('Ignore joint limits set to: ', calcObj.ignoreJointLimits);
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
  calcObj.step(deltaTime / 1000); // time step in seconds
  if (shutdownFlag === true) {
    self.postMessage({type: 'shutdown_complete'});
    console.log('main loop was finished')
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
// CdModuleを閉じ込めて、その関連オブジェクトを生成するhelper関数群
function createCdHelpers(module) {
  function makeCdDoubleVector(jsArray) {
    const vec = new module.DoubleVector();
    for (let i = 0; i < jsArray.length; ++i) {
      vec.push_back(jsArray[i]);
    }
    return vec;
  }
  function makeConvexShape(xyzArray) {
    // console.debug("module", module);
    const vec = new module.ConvexShape();
    for (let i = 0; i < xyzArray.length; ++i) {
      const xyz = xyzArray[i];
      vec.push_back({x: xyz[0], y: xyz[1], z: xyz[2]});
    }
    return vec;
  }
  // 他にも必要な関数を追加できる
  return {
    makeCdDoubleVector,
    makeConvexShape,
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

  const jointModelsArray = list.map(obj => {
    const xyz_in = obj.origin.$.xyz ?? [0,0,0];// [NaN, NaN, NaN];
    const xyz = modDoubleVector(mod,
				Array.isArray(xyz_in) && xyz_in.length === 3
				? xyz_in : [0,0,0]);//[NaN, NaN, NaN]);
    const rpy_in = obj.origin.$.rpy ?? [0,0,0];//[NaN, NaN, NaN];
    const rpy = modDoubleVector(mod,
				Array.isArray(rpy_in) && rpy_in.length === 3
				? rpy_in : [0,0,0]);//[NaN, NaN, NaN]);
    const axis_in = obj.axis.$.xyz ?? [0,0,1];//[NaN, NaN, NaN];
    const axis = modDoubleVector(mod,
				 Array.isArray(axis_in) && axis_in.length === 3
				 ? axis_in : [0,0,1]);//[NaN, NaN, NaN]);
    const jointModel = new mod.JointModelFlatStruct(axis, xyz, rpy);
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
    console.warn('Cycle detected or disconnected components in URDF joints');
  }
  return orderedJoints;
}

function updateLeaves(a, b) {
  for (const key in b) {
    if (!(key in a)) {
      console.debug('key in update.json:',key,' ignored');
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
      console.warn('key:',key,'val:',a[key],'is replaced by',bVal);
      a[key] = bVal; // 配列やオブジェクトでない値は上書き
    }
  }
  return a;
}
