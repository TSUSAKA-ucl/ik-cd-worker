'use client';
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
					 topicBridgeWebSocketURL})
{
  if (workerRef.current !== null) {
    console.error("Worker already exists.Something is wrong.");
  } else {
    console.log("******** Creating a new ik-cd-worker ********");
    workerRef.current = new Worker('/ik_cd_worker.js', { type: 'module',
							 name: robotName});
    console.log("workerRef.current: ", workerRef.current);
    workerRef.current.onmessage = (event) => {
      switch (event.data.type) {
      case 'ready': {
	const initMsg = { type: 'init',
			  filename: robotName +'/'+'urdf.json',
			  modifier: robotName +'/'+'update.json',
			  linkShapes: robotName +'/'+'shapes.json',
			  testPairs: robotName +'/'+'testPairs.json',
			  bridgeUrl: topicBridgeWebSocketURL
			};
	// console.warn('XXX init msg',initMsg);
	workerRef.current
	  .postMessage(initMsg);
      }
	break;
      case 'generator_ready':
	if (entity) {
	  entity.ikWorkerReady = true;
	  entity.emit('ik-worker-ready');
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
	  console.debug("Worker joint message:",
			event.data.joints.map(x => x.toFixed(3)).join(', '));
	  // Always skip to the latest data
	  workerData.current.joints = event.data.joints;
	}
	break;
      case 'status':
	workerData.current.status = event.data;
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


// *** class that sets the end effector point in the worker thread
export class ToolPointMover {
  constructor(workerRef) {
    this.workerRef = workerRef;
    import('aframe').then((AFRAME) => {
      this.THREE = AFRAME.THREE;
      this.toolPoint = new this.THREE.Vector3(0, 0, 0);
    });
  }
  _postToolPoint() {
    this.workerRef.current
      .postMessage({type: 'set_end_effector_point',
		    endEffectorPoint: this.toolPoint.toArray()});
    console.debug("Tool Point moved to: ", this.toolPoint.x.toFixed(3),
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
