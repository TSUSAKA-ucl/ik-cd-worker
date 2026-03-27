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
export default function IkWorkerManager({robotName,
					 entity,
					 initialJoints,
					 workerRef,
					 workerData,
					 cdWorkerRef={ready: false, el: null},
					 // currentはworker本体,
					 // readyはcd_workerの準備完了を表すフラグ,
					 // readyがtrueになっていればcd workerはonmessageでchannelを受け取れる状態, falseならreadyイベントの発火を待つ
					 // elはreadyイベントの発火元エンティティ。最低これを有効にしておかないとchannelの受け渡しができない可能性がある
					 topicBridgeWebSocketURL})
{
  if (workerRef.current !== null) {
    globalThis.__customLogger?.error("Worker already exists.Something is wrong.");
  } else {
    globalThis.__customLogger?.log('******** Creating a new ik-worker for',robotName,'********');
    if (entity?.sceneEl) {
      // cd workerがなければ作ってsceneElにつけ cdWorkerRefをセットする
      // cd workerが既にあれば cdWorkerRefをセットする
    }
    // ik_workerに対応するchannelを作り、渡す。反対側は既に作成済のcd_workerにpostMessageする
    channel = new MessageChannel();
    workerRef.current = new Worker('/ik_worker.js', { type: 'module',
						      name: robotName});
    globalThis.__customLogger?.debug("workerRef.current: ", workerRef.current);
    let isWaitingEndState = true;
    workerRef.current.onmessage = (event) => {
      switch (event.data.type) {
      case 'ready': {
	const channelTransferFunc = () => {
	  cdWorkerRef.current.postMessage({ type: 'add_port',
					    port: channel.port1,
					    from: robotName},
					  [channel.port1]);
	  workerRef.current.postMessage({ type: 'cd_port',
					  port: channel.port2,
					  to: robotName},
					[channel.port2]);
	};
	if (cdWorkerRef.ready) { channelTransferFunc();	} else {
	  cdWorkerRef.el?.addEventListener('cd-worker-ready', channelTransferFunc, { once: true });
	}
	const initMsg = { type: 'init',
			  filename: robotName +'/'+'urdf.json',
			  modifier: robotName +'/'+'update.json',
			  linkShapes: robotName +'/'+'shapes.json',
			  testPairs: robotName +'/'+'testPairs.json',
			  bridgeUrl: topicBridgeWebSocketURL
			};
	// globalThis.__customLogger?.warn('XXX init msg',initMsg);
	workerRef.current
	  .postMessage(initMsg);
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
