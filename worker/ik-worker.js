'use strict';
import { customLogger } from './customLogger.js';
globalThis.__customLogger = customLogger;
const ucl_logger = globalThis.__customLogger;
// ********************************
// デバッグコンソール出力
// if (typeof console.debug === 'function')  ucl_logger.debug = console.debug;
if (typeof console.log === 'function')  {
  ucl_logger.log = console.log;
  ucl_logger.info = console.log;
}
if (typeof console.warn === 'function')  ucl_logger.warn = console.warn;
if (typeof console.error === 'function')  ucl_logger.error = console.error;

// Worker script for handling messages and performing calculations
// worker state definition
import {TrapVelocGenerator} from './TrapVelocGenerator.js';
import { encode } from '@msgpack/msgpack';

import { st, sst, bridge } from './workerStateSingleton.js';
// import { updateLeaves, sortJointsByHierarchy } from './urdfUtils.js';
import { IkCdCalc } from './ikCdStepClass.js';
import MessageChannelHandler from './MessageChannelHandler.js';

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
// abIdはcd-worker用message channelがcd-workerに渡されると生成されるが、
// calcObj.abIdRegisteredは、shapesがregisterされた物だけabId数値がセットされる
calcObj.abIdRegistered = null;
calcObj.dontStepBack = false;
//
globalThis.base_coord = new Float64Array(16); // 基底座標を保存する配列
const global_base_coord = globalThis.base_coord; // グローバルにアクセスできるようにする
// global_base_coordは、global_base_coord[15] < 0のとき未定義とする
for (let i = 0; i < 15; i++) {
  global_base_coord[i] = (i % 5 === 0) ? 1 : 0; // i%5===0のときは1, それ以外は0
}
global_base_coord[15] = -1; // 最初は未定義


let workerInitialized = false;
const commandQueue = [];
// ******** worker message handler ********
// 現在、onmessageハンドラーで直接処理していものを、commandQueueを
// stepの後にpollingで処理する形に変更する。
// 以下のコマンドはqueueに積まずに直接実行する。
// shutdown, set_worker_loglevel, set_slrm_loglevel, cd_port
// それ以外のコマンドはqueueに積み、さらに'init'コマンドがresolveするまで
// 他のコマンドは待つ。
// destination, set_joint_targets, slow_rewindの3種のmoveコマンドは
// さらにmoveCommandQueueに積んで、subStateがdormantあるいはconvergedになったら
// 次のコマンドを処理(subStateを変更)する。この処理はIkCdCalcクラスの
// calcObjの中で行う

ucl_logger?.debug('now setting onmessage')
self.onmessage = function(event) {
  const data = event.data;
  // if (data.type !== 'set_base_coord' &&
  //     data.type !== 'destination' &&
  //     data.type !== 'set_joint_targets' &&
  //     data.type !== 'slow_rewind') {
  // if (data.type === 'stop_dependency') {
  //   ucl_logger?.debug('## Received stop_dependency or XXX command ',
  // 		    'type:', data.type,
  // 		    'self abId:', self.abId,
  // 		    'for stopAbId:', data.stopAbId
  // 		   );
  // }

  switch (data.type) {
  case 'shutdown': // workerを終了する
    workerInitialized = false; // 新たなコマンドの処理を止めるためにフラグを下ろす
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
    // self.cdPortHandler.callRpcでcd-workerにremove_abコマンドを送る
    if (self.cdWorkerPort) {
      self.cdWorkerPort.close();
      self.cdWorkerPort = null;
    }
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
    self.cdPortHandler = new MessageChannelHandler(self.cdWorkerPort);
    calcObj.cdPortHandler = self.cdPortHandler; // calcObjからもアクセスできるようにする
    // setNewDataは、repareRewindQueue()の中で定義される関数であるため、
    // ここでは定義できない
    //   switch (event.data.command) {
    //   case 'collision_pairs':
    // 	if (typeof calc.setNewData === 'function') {
    // 	  calcObj.setNewData(event.data.sequence, event.data.rbIds);
    // 	}
  }
    break;
  default:
    // if (data.type === 'stop_dependency') {
    //   ucl_logger?.debug('## RECEIVED stop_dependency command for abId:', data.stopAbId,
    // 			'current abId:', self.abId
    // 		       );
    // }
    // enqueue other commands
    // ただしdata.type==='init'のときはqueueの先頭に積む。
    if (data.type === 'init') {
      commandQueue.unshift(data); // initは優先して処理するためqueueの先頭に積む
    } else {
      commandQueue.push(data);
    }
    break;
  }
};

async function processCommandQueue() {
  let cmdVelGen = calcObj.cmdVelGen; // CmdVelGeneratorのインスタンスを保持する変数
  const { makeDoubleVector } = createHelpers(SlrmModule);
  while (commandQueue.length > 0) {
    // const data = commandQueue.shift();
    // dequeueするまえにinitかどうかをチェックしてinitならば処理する。
    // initでなければworkerInitializedがtrueになるまで保留
    // commandQueue.length > 0なのでcommandQueue[0]を見る
    if (commandQueue[0].type !== 'init' && !workerInitialized) {
      // const data = commandQueue.shift(); // NG
      // enqueue側でinitは先頭に積むようにしているので他のコマンドは残しておく
      break; // initでないコマンドが来ていてworkerが初期化されていないときは、commandQueueの先頭を処理せずに待つ
    } else {
      const data = commandQueue.shift();
      // if (data.type === 'stop_dependency') {
      // 	ucl_logger?.debug('## RECEIVED stop_dependency COMMAND for abId:', data.stopAbId,
      // 			  'current abId:', self.abId);
      // }
      // if (data.type !== 'destination') {
      // 	ucl_logger?.debug('Processing command from queue:', data);
      // }
      switch (data.type) {
      case 'init': if (calcObj.state === st.waitingRobotType) {
	calcObj.state = st.generatorMaking;
	ucl_logger?.log('constructing CmdVelGenerator with :',
			data.filename);
	ucl_logger?.debug('URDF modifier file is', data.modifier);
	// 初期化処理
	const
	finalPromise = fetch(data.filename)
	  .then(response => response.json())
	  .then(async jsonData => {
	    let urdfIsSorted = false;
	    let urdfData = null;
	    if (Array.isArray(jsonData)) {
	      urdfData = {...jsonData};
	      urdfIsSorted = true;
	    } else {
	      urdfData = jsonData;
	    }
	    const
	    modifierPromise =
	      fetch(data.modifier)
	      .then(response => response.json())
	      .then(modifierData => {
		updateLeaves(urdfData, modifierData);
	      });
	    await modifierPromise; // modifierの適用が完了するまで待つ
	    urdfData = Object.values(urdfData);
	    if (!urdfIsSorted) {
	      urdfData = sortJointsByHierarchy(urdfData);
	    }

	    // setting the WASM object's parameters
	    const {jointModelVector,
		   jointModelsArray} = createJointModel(SlrmModule, urdfData);
	    ucl_logger?.debug('type of SlrmModule.CmdVelGen: '
			      + typeof SlrmModule.CmdVelGenerator);
	    calcObj.cmdVelGen = new SlrmModule.CmdVelGenerator(jointModelVector);
	    cmdVelGen = calcObj.cmdVelGen;
	    cmdVelGen.heapF64 = SlrmModule.HEAPF64;
	    jointModelsArray.forEach(model => model.delete());
	    jointModelVector.delete();
	    if (cmdVelGen === null || cmdVelGen === undefined) {
	      ucl_logger?.error('generation of CmdVelGen instance failed');
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

	    // commandQueueのdataのparametersに干渉形状の定義があれば
	    // fetchしてcdWorkerに送る
	    if (data.linkShapes && self.cdPortHandler) {
	      ucl_logger?.log('## fetching link shapes from: ', data.linkShapes);
	      const
	      linkShapesPromise = fetch(data.linkShapes)
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
		  ucl_logger?.log('NOW preparing the PACKED SHAPE DATA for',
				  data.linkShapes);
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
		  ucl_logger?.log('NOW Packed data was created for',
				 self.myBodyName);

		  // fetch test pairs from data.testPairs if exists
		  let testPairs = [];
		  if (!data.testPairs) {
		    for (let i=0; i< linkShapes.length-4; i++) {
		      for (let j=i+2; j<linkShapes.length; j++) {
			testPairs.push([i,j]);
		      }
		    }
		    ucl_logger?.log('using default test pairs: ',
				      testPairs);
		  } else {
		    ucl_logger?.log('fetch test pairs from',
				      data.testPairs);
		    const response = await fetch(data.testPairs);
		    testPairs = await response.json();
		  }
		  packed.testPairs = testPairs;
		  if (self.cdPortHandler) {
		    ucl_logger?.log('NOW calling RPC for', self.myBodyName);
		    const abIdObj = await
		    self.cdPortHandler.callRpc({command: 'link_shapes',
					       shapes: packed,
					       name: self.myBodyName},
					      [packed.abLayer.buffer,
					       packed.rbLayer.buffer,
					       packed.saLayer.buffer,
					       packed.vertices.buffer],
					     500); // 0.5秒のタイムアウトを設定
		    self.abId = abIdObj.abId; // cd-workerから返されたabIdを保存
		    calcObj.abIdRegistered = self.abId; // calcObjからもアクセスできるようにする
		  } else {
		    throw new Error('cdPortHandler is not ready to send link shapes data');
		  }
		})
		.catch(error => {
		  ucl_logger?.error('Error SHAPE file:',
				    error);
		});
	      await linkShapesPromise; // 干渉形状の送信が完了するまで待つ
	    }
	    if (data.bridgeUrl) {
	      ucl_logger?.debug('receive bridge URL: ', data.bridgeUrl);
	      // bridge用のURLが付いているためbridgeが使える
	      // workerStateSingletonのbridgeオブジェクトにURLをセットし
	      // 接続して、bridge.messageQueueをsendする
	      bridge.url = data.bridgeUrl;
	      bridge.connect(); // 
	    }
	    // なにかの加減でオブジェクト生成に失敗した場合はここでエラーがthrownされる
	    calcObj.state = st.generatorReady;
	    ucl_logger?.log('## sending generator_ready message with abId:', calcObj.abIdRegistered);
	    self.postMessage({type: 'generator_ready', ab_id: calcObj.abIdRegistered});
	  })
	  .catch(error => {
	    ucl_logger?.warn('Err fetching or parsing URDF.JSON file:',
			     error);
	    ucl_logger?.warn('URDF file name:', data.filename);
	    // 新たな'init'を受け付けるために状態をwaitingRobotTypeに戻す
	    calcObj.state = st.waitingRobotType;
	  });
	await finalPromise; // ここで初期化処理全体が完了するまで待つ
	ucl_logger?.debug('Initialization complete, finalPromise resolved');
	workerInitialized = true; // 初期化が完了したことを示すフラグを立てる
      }
	break;

      case 'ignore_pairs':
	ucl_logger?.debug('$$$$$$$$ Received ignore pairs command:', data.ignorePairs);
	ucl_logger?.debug('$$$$$$$$ cdPortHandler ready:', !!self.cdPortHandler, 'abId:', self.abId);
	ucl_logger?.debug('$$$$$$$$ Current state:', calcObj.state);
	if (calcObj.state === st.slrmReady || calcObj.state === st.generatorReady) {
	  // ik-workerは、my_abId:my_link, abId:link型(整数4個)をパックしたtyped arrayにしてcd-workerにRPCする(command新設)
	  // ここで受け取っている event.data.ignorePairsは
	  // ignorePairs: [{ myLink: pair.myLink,	otherAbId: otherEl.abId, otherLink: pair.otherLink},...]
	  // el.workerRef?.current?.postMessage({ type: 'ignore_pairs', ignorePairs }) :
	  if (self.cdPortHandler && typeof self.abId === 'number') {
	    const ignorePairs = new Int32Array(data.ignorePairs.length * 4);
	    data.ignorePairs.forEach((pair, index) => {
	      ignorePairs[index*4] = self.abId; // my_abId
	      ignorePairs[index*4 + 1] = pair.myLink;
	      ignorePairs[index*4 + 2] = pair.otherAbId; // other_abId
	      ignorePairs[index*4 + 3] = pair.otherLink;
	    } );
	    try {
	      ucl_logger?.debug('$$$$$$$$$$ Sending ignore pairs to cd-worker:', data.ignorePairs);
	      await self.cdPortHandler?.callRpc({command: 'ignore_pairs', ignorePairs},
		[ignorePairs.buffer], 500);
	      ucl_logger?.debug('Sent ignore pairs to cd-worker:', data.ignorePairs);
	    } catch (error) {
	      ucl_logger?.error('Failed to send ignore pairs to cd-worker:', error);
	    }
	  } else {
	    ucl_logger?.error('cdPortHandler is not ready to send ignore pairs data');
	  }
        }
	break;

      case 'set_initial_joints':
	if (calcObj.state === st.generatorReady ||
	    calcObj.state === st.slrmReady) {
	  if (data.joints) {
	    // 初期ジョイントの設定処理
	    const joints = new Float64Array(data.joints.length);
	    joints.set(data.joints);
	    calcObj.joints = joints;
	    // prepareRewindQueueu()の前にcalcObj.jointsのセットが必要
	    calcObj.prepareRewindQueue(); // ここでcdWorkerPortの
	    // onmessageハンドラに付けるcalcObj.setNewData()関数が
	    // 使用できるようになる
	    self.cdPortHandler?.on('collision_pairs', calcObj.setNewData);
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
	    // cd-workerに初期rb座標系を送る
	    calcObj.sendLinkCoordsToCd(calcObj.joints);
	  }
	} break;
      case 'stop_dependency':
	// 引数 data.stopAbId このabIdの衝突が検出されたら本abも停止(rewindQueue)する
	// cdWorkerに'stop_dependency'でcallRpcすればcdWorkerが自動的に処理し、stopAbId衝突の場合も
	// 本ab(ik-worker)に衝突情報を送ってくる
	// ucl_logger?
	  console.log('## stop_dependency command Received for abId:', data.stopAbId,
			'current abId:', self.abId);
	if (self.cdPortHandler && typeof self.abId === 'number') {
	  try {
	    await self.cdPortHandler?.callRpc({command: 'stop_dependency', stopAbId: data.stopAbId},
					      null, 500);
	    ucl_logger?.info('Sent stop_dependency command to cd-worker for abId:', data.stopAbId);
	  } catch (error) {
	    ucl_logger?.error('Failed to send stop_dependency command to cd-worker:', error);
	  }
	} else {
	  ucl_logger?.error('cdPortHandler is not ready to send stop_dependency command');
	}
	break;
      case 'set_base_coord':
	// まず引数のdata.baseCoordが正しいか確認する。サイズ16でbaseCoord[15]>0ならば有効とみなす。
	if (!data.baseCoord || data.baseCoord.length !== 16 || data.baseCoord[15] <= 0) {
	  ucl_logger?.error('Invalid baseCoord data received:', data.baseCoord);
	} else {
	  // cmdVelGenがemscriptenのmodule factoryの生成物の場合は、ここで直接セットする。
	  // cmdVelGenにセットしたらglobal_base_coordは未定義にする
	  // cmdVelGenがemscriptenのオブジェクトで無い場合は, global_base_coordを更新して、
	  // calcObjのstep関数内でcmdVelGenにセットする。
	  if (cmdVelGen && typeof cmdVelGen.notifyWTBaseBufferUpdated === 'function') {
	    const ptr = cmdVelGen.getWTBaseBufferPtr();
	    const size = cmdVelGen.getWTBaseBufferSize();
	    if (ptr && size) {
	      const wTbaseArray = new Float64Array(SlrmModule.HEAPF64.buffer, ptr, size);
	      wTbaseArray.set(data.baseCoord);
	      ucl_logger?.debug('XXXXX Base coordinate set to CmdVelGen buffer: ' +
				data.baseCoord.slice(12,15).map(v => v.toFixed(3)).join(', ')+
				' ...');
	      cmdVelGen.notifyWTBaseBufferUpdated();
	      global_base_coord.set(data.baseCoord);
	      global_base_coord[15] = -1; // 使用済、未定義にする
	      ucl_logger?.debug('Base coordinate updated: ' +
				data.baseCoord.slice(0,4).map(v => v.toFixed(3)).join(', ')+
				' ...');
	      // リンクのワールド座標値が変化するため,calcObjのsendLinkCoordsToCd()を呼んでおく
	      if (calcObj.state === st.slrmReady && calcObj.subState !== sst.dormant) {
		calcObj.sendLinkCoordsToCd(calcObj.joints);
	      }
	    } else {
	      ucl_logger?.error('Failed to get wTbase buffer pointer or size');
	    }
	  } else { // cmdVelGenが存在しないか、notifyWTBaseBufferUpdatedが関数でない 
	    global_base_coord.set(data.baseCoord);
	    ucl_logger?.debug('Base coordinate stored in global_base_coord: ' +
			      data.baseCoord.slice(0,4).map(v => v.toFixed(3)).join(', ') +
			      ' ...');
	  }
	}
	break;
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
	    moveCommandQueue.push({ type: 'jMoveVelocity',
				    velocityLimit: Math.PI });
	    moveCommandQueue.push({ type: 'jMoveGain',
				    gain: 100 });
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
	if (makeDoubleVector) {

	  if (data.endEffectorPoint &&
	      data.endEffectorPoint.length === 3 &&
	      typeof data.endEffectorPoint[0] === 'number' &&
	      typeof data.endEffectorPoint[1] === 'number' &&
	      typeof data.endEffectorPoint[2] === 'number') {
	    const endEffectorPosition = makeDoubleVector(data.endEffectorPoint);
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
	    const endEffectorOrientation = makeDoubleVector(eigenQuat);
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
		  = makeDoubleVector(data.velocityLimit);
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
	    // calcObj.ignoreCollision = data.ignoreCollisions;
	    if (calcObj.rewindQueue) {
	      calcObj.rewindQueue.ignore = data.ignoreCollisions;
	      ucl_logger?.log('Ignore collisions set to: ',
			      calcObj.rewindQueue.ignore);
	      if (data.ignoreCollisions) {
		// calcObj.resetRewindQueue(); // 現在のjoint値でqueueをリセットする
		// destinationを消さないとqueueの最後のjoint値にジャンプする
		calcObj.noDestination = true;
	      }
	    } else {
	      ucl_logger?.error('set_ignore_collisions: rewindQueue is not ready to set ignoreCollisions');
	    }	      
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
              // calcObj.joints.lengthサイズの全て1のArrayのmaskを作成して
              // calcObj.setJointLimitIgnoreMask(mask)を呼ぶ
              const mask = new Array(calcObj.joints.length).fill(1);
              calcObj.setJointLimitIgnoreMask(mask);
	      // calcObj.jointLimitKeepMoving = data.jointLimitKeepMoving;
	      ucl_logger?.log('Joint limit keep moving mask set to: ',
                calcObj.jointLimitIgnoreMask);
	    }
          }
	break;
      case 'set_joint_limit_keep_moving_mask':
        if (calcObj.state === st.generatorReady ||
          calcObj.state === st.slrmReady) {
            if (data.jointLimitKeepMovingMask !== undefined &&
              Array.isArray(data.jointLimitKeepMovingMask) &&
              data.jointLimitKeepMovingMask.length === calcObj.joints.length) {
                // calcObj.jointLimitKeepMovingMask = data.jointLimitKeepMovingMask;
                calcObj.setJointLimitIgnoreMask(data.jointLimitKeepMovingMask);
                ucl_logger?.log('Joint limit keep moving mask set to: ',
                  calcObj.jointLimitIgnoreMask);
              } else {
                ucl_logger?.error('set_joint_limit_keep_moving_mask: invalid mask data received:', data.jointLimitKeepMovingMask, 'expected length:', calcObj.joints.length);
              }
          }
        break;
      default:
	  break;
      }
    }
  }
}


// ******** main loop ********
// const timeInterval = 4; // time step for simulation in milliseconds
// const logInterval = 0n/BigInt(timeInterval); // log interval in BigInt
// // ******** control parameters ********
// let counter = 0n;
// let jMoveGain = 10.0; // joint move command gain
// let jMoveVelocityLimit = Math.PI/3.0; // joint move command velocity limit rad/s



// ******** worker main loop ********
const loopIntervalMs = 4
// let nextLoopTime = performance.now();
// let prevTime = performance.now();
async function mainLoop() {
  const now = performance.now();
  const deltaTime = now - prevTime;
  prevTime = now;
  nextLoopTime += loopIntervalMs;
  // ここにstepの上限をつけること。stepが長くなりすぎると速度フィードバックが破綻する
  if (calcObj.state === st.generatorReady ||
      calcObj.state === st.slrmReady) {
    // urdf.jsonがfetchされずgeneratorReadyだがslrmReadyでない
    // ケースは正しく実装されていないため、今のところgeneratorReadyのときはstepしないようにする
    calcObj.step(deltaTime); // time step in milliseconds
  }
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
  await processCommandQueue(); // コマンドキューの処理が完了するまで待つ
  const delay = Math.max(0, nextLoopTime - performance.now());
  setTimeout(mainLoop, delay);
}

// ******************************
// ******** worker start ********
// ******************************
calcObj.state = st.waitingRobotType;
self.postMessage({type: 'ready'});
//
calcObj.timeInterval = loopIntervalMs;
let nextLoopTime = performance.now();
let prevTime = performance.now();

mainLoop(); // main loopを開始


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
