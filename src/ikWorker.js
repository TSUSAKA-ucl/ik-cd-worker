import AFRAME from 'aframe';
// const THREE = window.AFRAME.THREE;
import IkWorkerManager from './IkWorkerManager.js';
import { withObjReady } from './withObjReady.js';

AFRAME.registerComponent('ik-worker', {
  schema: { type: 'array'}, // intial joint value
  init: function() {
    this.el.addEventListener('robot-dom-ready', () => {
      // ****************
      // Worker thread management
      this.el.workerRef = {current: null};
      const workerRef = this.el.workerRef;
      this.el.workerData = {current: { joints: null,
				       status: {}, pose: {} }};
      const workerData = this.el.workerData;
      const initialJoints = this.data.map(parseFloat);
      // *** controller offset subscribe
      // const bridgeProtocol = location.protocol==='https:' ? 'wss:':'ws:';
      // const bridgePort = 9090;
      const topicBridgeWebSocketURL =
	    // `${bridgeProtocol}//${location.hostname}:${bridgePort}`;
	    null;
      console.log('UUUUUUU call IkWorkerManager with model',this.el.model);
      this.robotRegistryFunc = null;
      this.el.addEventListener('ik-worker-ready', () => {
	this.robotRegistryFunc = () => {
	  const id = this.el.id;
	  const robotRegistryComp = this.el.sceneEl.robotRegistryComp;
	  robotRegistryComp.add(id, {worker: this.el.workerRef,
				     workerData: this.el.workerData});
	  console.log('Robot ', id, ' worker added:', this.el.workerRef);
	  this.el.emit('ik-worker-start'); // what do i do next?
	};
	this.robotRegistryFunc();
      });
      this.removeWorker = IkWorkerManager({robotName: this.el.model,
					   entity: this.el,
					   initialJoints,
					   workerRef,
					   workerData,
					   topicBridgeWebSocketURL});
      // This robot may be or may NOT be REGISTERED in 'robot-registry'
      // before the emission of 'robot-dom-ready' by urdfLoader2
      // Here, use the add function to register it in the registry.
      // if (this.el.sceneEl.hasLoaded) {
      // if (this.el.model) {
      // 	registerRobotFunc();
      // } else {
      // 	// this.el.sceneEl.addEventListener('loaded', registerRobotFunc,
      // 	this.el.addEventListener('robot-dom-ready', registerRobotFunc,
      // 				 { once: true });
      // }
    }, {once: true});
  },
  remove:function() {
    // if (this?.remove) this.remove();
  }
});

AFRAME.registerComponent('joint-weight', {
  schema: {
    override: { type: 'string', default: ''},
  },
  parseAndSetMap: function() {
    this.map = {};
    this.data.override.split(',').forEach( pairStr => {
      const [jointName, weightStr] = pairStr.split(':');
      const weight = parseFloat(weightStr);
      if (jointName && !isNaN(weight)) {
	if (typeof parseInt(jointName) === 'number'
	    && !isNaN(parseInt(jointName))) {
	  this.map[jointName] = weight;
	} else {
	  console.warn(`Invalid joint name for joint-weight: ${jointName}`);
	}
      }
    });
  },
  init: function() {
    this.parseAndSetMap();
    this.setJointWeight = (map) => {
      if (this.el.workerRef?.current) {
	Object.entries(map).forEach( ([jointName, weight]) => {
	  const msg = { type: 'set_joint_weights',
			jointNumber: parseInt(jointName),
			jointWeight: weight };
	  this.el.workerRef.current.postMessage(msg);
	});
      }
    };
    withObjReady(this.el, 'ik-worker-ready',
		 this.el.ikWorkerReady,
		 this.setJointWeight,
		 this.map);
  },
  update: function() {
    this.parseAndSetMap();
    this.setJointWeight(this.map);
  }
});

AFRAME.registerComponent('joint-desirable', {
  schema: {
    upper: { type: 'string', default: ''},
    lower: { type: 'string', default: ''},
    gain: { type: 'string', default: ''},
  },
  parseJointMap: function (map, dataStr) {
    dataStr.split(',').forEach( pairStr => {
      const [name, value] = pairStr.split(':');
      const valNum = parseFloat(value);
      if (name && !isNaN(valNum)) {
	map[name] = valNum;
      } else {
	console.error(`Invalid joint desirable setting: ${pairStr}`);
      }
    });
  },
  parseAndSetDesirable: function() {
    this.desirable = {};
    const upperMap = {};
    const lowerMap = {};
    const gainMap = {};
    this.parseJointMap(upperMap, this.data.upper);
    this.parseJointMap(lowerMap, this.data.lower);
    this.parseJointMap(gainMap, this.data.gain);
    Object.entries(gainMap).forEach( ([jointName, gain]) => {
      const upper = upperMap[jointName] ? upperMap[jointName] : 2*Math.PI;
      const lower = lowerMap[jointName] ? lowerMap[jointName] : -2*Math.PI;
      this.desirable[jointName] = { upper, lower, gain };
    });
    Object.entries(upperMap).forEach( ([jointName, upper]) => {
      if (!this.desirable[jointName]) {
	console.warn(`Upper desirable without gain for joint: ${jointName} keep ignored.`);
      }
    });
    Object.entries(lowerMap).forEach( ([jointName, lower]) => {
      if (!this.desirable[jointName]) {
	console.warn(`Lower desirable without gain for joint: ${jointName} keep ignored.`);
      }
    });
  },
  init: function() {
    this.parseAndSetDesirable();
    this.setJointDesirable = (desirable) => {
      if (this.el.workerRef?.current) {
	const loglevel = { type: 'set_slrm_loglevel',
			   logLevel: 4 };
	this.el.workerRef.current.postMessage(loglevel);
	Object.entries(desirable).forEach( ([jointName, descObj]) => {
	  const msg = { type: 'set_joint_desirable',
			jointNumber: parseInt(jointName),
			upper: descObj.upper,
			lower: descObj.lower,
			gain: descObj.gain };
	  console.log('in AF component desirable postMessage:',msg);
	  this.el.workerRef.current.postMessage(msg);
	});
      }
    };
    withObjReady(this.el, 'ik-worker-ready',
		 this.el.ikWorkerReady,
		 this.setJointDesirable,
		 this.desirable);
  },
  update: function() {
    this.parseAndSetDesirable();
    this.setJointDesirable(this.desirable);
  }
});
