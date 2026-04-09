'use client';
import { customLogger } from './customLogger.js';
globalThis.__customLogger = customLogger;
// ****************
// Worker thread manager component
// const workerRef = useRef(null);
// const initialJoints = [0, 0, 0, 0, 0, 0];
// const robotName = "ur5e";
// const 
// const workerLastJoints = workerData.current.joints;
// const workerLastStatus = workerData.current.status;
// const workerLastPose = workerData.current.pose;
export default async function IkWorkerManager({robotName,
					       entity,
					       initialJoints,
					       workerRef,
					       workerData,
					       cdWorkerRef={ready: false, el: null},
					       // currentはworker本体,
					       // readyはcd-workerの準備完了を表すフラグ,
					       // readyがtrueになっていればcd workerはonmessageでchannelを受け取れる状態, falseならreadyイベントの発火を待つ
					       // elはreadyイベントの発火元エンティティ。通常はsceneEl。最低これを有効にしておかないとchannelの受け渡しができない可能性がある
					       topicBridgeWebSocketURL})
{
  if (workerRef.current !== null) {
    globalThis.__customLogger?.error("Worker already exists.Something is wrong.");
  } else {
    globalThis.__customLogger?.log('******** Creating a new ik-worker for',robotName,'********');
    // sceneElが存在してcd workerが存在しない場合は無条件でcd workerを作る。
    // ただし、ik workerからlink_shapes commandが来なければ何も計算しない
    // sceneElが無ければcd workerもchannelも作れないが、ik workerは作れる
    // sceneEl.systems.cdWorkerが無ければ、cd workerは生成済でないため、cd workerを作る。
    // cd workerを作成したらsceneEl.systemsにセットして、cd_worker_readyメッセージを待つ
    // cd_worker_readyメッセージ受信がresolveしたら message channel作成とik worker作成に進む
    //
    // cdWorkerが生成される => sceneEl.systems.cdWorker.currentにWorkerオブジェクトが入る    
    // 生成したタスクはPromiseでラップして、readyになるのを待つ
    // promiseは他のタスクと共有するためsceneEl.systems.cdWorker.promiseに入れる
    // 待っている間に、他のタスクはcdWorkerを生成せず同じpromiseを待つようにする
   if (entity?.sceneEl) {
      if (!entity.sceneEl.systems?.cdWorker) {
	try {
	  globalThis.__customLogger?.warn('Creating a new cd-worke: sceneEl.systems', entity.sceneEl.systems);
	  entity.sceneEl.systems.cdWorker = { current: null, ready: false, el: entity.sceneEl };
	  entity.sceneEl.systems.cdWorker.promise = new Promise((resolve, reject) => {
	    globalThis.__customLogger?.warn('Creating cd-worker...');
	    const cdWorker = new Worker('/cd-worker.js', { type: 'module', name: 'cd-worker'});
	    entity.sceneEl.systems.cdWorker.current = cdWorker;
	    cdWorkerRef = entity.sceneEl.systems.cdWorker;
	    globalThis.__customLogger?.warn('cd-worker created, waiting for ready message...');
	    cdWorker.onmessage = (event) => {
	      if (event.data.type === 'cd_worker_ready') {
		globalThis.__customLogger?.debug('cd-worker is ready');
		cdWorkerRef.ready = true;
		cdWorkerRef.el = entity.sceneEl; // readyイベントの発火元エンティティを保存
		resolve();
	      } else if (event.data.type === 'wasm_error') {
		globalThis.__customLogger?.error('cd-worker failed to initialize WASM module:', event.data.error);
		entity.sceneEl.systems.cdWorker = null;
		reject(new Error('cd-worker failed to initialize WASM module: ' + event.data.error));
	      }
	    };
	    cdWorker.onerror = (error) => {
	      globalThis.__customLogger?.error('Failed to load cd-worker:', error);
	      entity.sceneEl.systems.cdWorker = null;
	      reject(error);
	    }
	  });
	  // await createCdWorker();
	  await entity.sceneEl.systems.cdWorker.promise;
	} catch (error) {
	  globalThis.__customLogger?.error('Error during cd-worker creation:', error);
	}
      } else {
	globalThis.__customLogger?.debug('typeof cd-worker object: ', typeof entity.sceneEl.systems.cdWorker.current);
	if (entity.sceneEl.systems.cdWorker.current instanceof Worker) {
	  cdWorkerRef = entity.sceneEl.systems.cdWorker;
	  await entity.sceneEl.systems.cdWorker.promise;
	  globalThis.__customLogger?.debug('cd-worker already exists');
	}
      }
    }
    // sceneElが無い場合はcd workerもchannelも作れないが、ik workerは作れる
    // この時点で、cdWorkerが存在していれば readyであるはず。

    workerRef.current = new Worker('/ik-worker.js', { type: 'module',
						      name: robotName});
    globalThis.__customLogger?.debug("workerRef.current: ", workerRef.current);
    let isWaitingEndState = true;
    workerRef.current.onmessage = (event) => {
      switch (event.data.type) {
      case 'ready': {
	const channelTransferFunc = () => {
	  // ik-workerに対応するchannelを作り、渡す。反対側は既に作成済のcd_workerにpostMessageする
	  const channel = new MessageChannel();
	  cdWorkerRef.current.postMessage({ type: 'add_port',
					    port: channel.port1,
					    from: robotName},
					  [channel.port1]);
	  globalThis.__customLogger?.warn('Message channel created and port1 sent to cd-worker');
	  workerRef.current.postMessage({ type: 'cd_port',
					  port: channel.port2,
					  from: robotName},
					[channel.port2]);
	};
	
	if (cdWorkerRef.ready) {
	  channelTransferFunc(); 
	} else {
	  globalThis.__customLogger?.warn('cd-worker not ready yet, waiting for ready event...');
	}
	const initMsg = { type: 'init',
			  filename: robotName +'/'+'urdf.json',
			  modifier: robotName +'/'+'update.json',
			  linkShapes: robotName +'/'+'shapes.json',
			  testPairs: robotName +'/'+'testPairs.json',
			  bridgeUrl: topicBridgeWebSocketURL
			};
	// globalThis.__customLogger?.warn('XXX init msg',initMsg);
	workerRef.current.postMessage(initMsg);
	// もし entity.object3Dの値があれば 'type: set_base_coord'をpostMessage
	if (entity?.object3D) {
	  // 念の為matrixWorldを更新してから値を取る
	  entity.object3D.updateMatrixWorld();
	  const baseCoord = entity.object3D.matrixWorld.elements;
	  workerRef.current.postMessage({ type: 'set_base_coord',
					  baseCoord: baseCoord });
	}
      }
	break;
      case 'generator_ready':
	if (entity) {
	  entity.ikWorkerReady = true;
	  entity.emit('ik-worker-ready', null, false);
	}
	workerRef.current
	  .postMessage({ type: 'set_exact_solution',
			 exactSolution: false });
	workerRef.current
	  .postMessage({ type: 'set_initial_joints',
			 joints: initialJoints,
		       });
	break;
      case 'joints':
	if (event.data.joints) {
	  globalThis.__customLogger?.debug("Worker joint message:",
			event.data.joints.map(x => x.toFixed(3)).join(', '));
	  // Always skip to the latest data
	  workerData.current.joints = event.data.joints;
	}
	break;
      case 'status':
	workerData.current.status = event.data;
	if (isWaitingEndState &&
	    workerData.current.status.status === 'END') {
	  entity.emit('ik-worker-arrival', null, false);
	  isWaitingEndState = false;
	}
	if (workerData.current.status.status !== 'END') {
	  isWaitingEndState = true;
	}
	break;
      case 'pose':
	workerData.current.pose = event.data;
	break;
      }
    };
  }
  //
  return () => {
    if (workerRef.current) {
      workerRef.current.terminate();
      workerRef.current = null;
    }
  };
}

import AFRAME from 'aframe';
const THREE = AFRAME.THREE;

// *** class that sets the end effector point in the worker thread
export class ToolPointMover {
  constructor(workerRef) {
    this.workerRef = workerRef;
    // import('aframe').then((AFRAME) => {
    //   this.THREE = AFRAME.THREE;
    //   this.toolPoint = new this.THREE.Vector3(0, 0, 0);
    // });
    // == alternative ==
    this.toolPoint = new THREE.Vector3(0, 0, 0);
  }
  _postToolPoint() {
    this.workerRef.current
      .postMessage({type: 'set_end_effector_point',
		    endEffectorPoint: this.toolPoint.toArray()});
    globalThis.__customLogger?.debug("Tool Point moved to: ", this.toolPoint.x.toFixed(3),
		  this.toolPoint.y.toFixed(3), this.toolPoint.z.toFixed(3));
  }
  delta(delta) {
    if (typeof delta === 'number') {
      this.toolPoint.z += delta;
      this._postToolPoint();
    }
  }
  reset() {
    this.toolPoint.x = 0; this.toolPoint.y = 0; this.toolPoint.z = 0;
    this._postToolPoint();
  }
  set(position) {
    this.toolPoint.copy(position);
    this._postToolPoint();
  }
  get() {
    return this.toolPoint;
  }
}
