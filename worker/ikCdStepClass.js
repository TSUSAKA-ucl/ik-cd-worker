// Worker script for handling messages and performing calculations
// worker state definition
// import {TrapVelocGenerator} from './TrapVelocGenerator.js';
import { encode } from '@msgpack/msgpack';

import { st, sst, bridge } from './workerStateSingleton.js';

// function  copyArrayToVec(jsArray, emVec) {
//   const emVecSize = emVec.size();
//   for (let i = 0; i < emVecSize; ++i) {
//     emVec.set(i, jsArray[i]);
//   }
// }
function copyArrayToWasmVec(jsArray, emVec) { // , wasmModule) {
  // const ptr = wasmModule.getDoubleVecData(emVec);
  // const wasmView = wasmModule.HEAPF64.subarray(ptr>>3,
  // 					       (ptr>>3) + emVec.size());
  // wasmView.set(jsArray);
  for (let i = 0; i < emVec.size(); ++i) {
    emVec.set(i, jsArray[i]);
  }
}
function copyWasmVecToArray(emVec, jsArray, wasmModule) {
  // const ptr = wasmModule.getDoubleVecData(emVec);
  // const wasmView = wasmModule.HEAPF64.subarray(ptr>>3,
  // 					       (ptr>>3) + jsArray.length);
  // jsArray.set(wasmView);
  for (let i = 0; i < jsArray.length; ++i) {
    jsArray[i] = emVec.get(i);
  }
}

class IkCdCalc {
  constructor (slrmModule, cdModule) {
    // this.jointLimitKeepMoving = true;
    this.jointLimitKeepMoving = false;
    this.slrmModule = slrmModule; // SLRM WASM module
    this.cdModule = cdModule;     // Collision Detection WASM module
    this.state =  st.initializing; // worker state
    this.subState =  sst.dormant;  // slrm & joint mover/rewinder state
    this.timeInterval = 4; // time step for simulation in milliseconds
    this.logInterval = 0n/BigInt(this.timeInterval); // log interval in BigInt
    // ******** flags ********
    this.noDestination = true; // 目標位置姿勢が存在しないかどうか
    // this.newDestinationFlag = false; // 新しいdestinationが来たかどうか
    this.exactSolution = false; // singularity通過のための設定
    this.ignoreCollision = false; // 干渉判定を無視するかどうか
    this.ignoreJointLimits = false; // ジョイントリミットを無視するかどうか
    // ******** wasm objects (set by the onmessage handler) ********
    this.cmdVelGen = null;
    this.gjkCd = null;
    // ******** control parameters ********
    this.counter = 0n;
    this.jMoveGain = 10.0; // joint move command gain
    this.jMoveVelocityLimit = Math.PI/3.0; // joint move command velocity limit rad/s
    // ******** result variables ********
    this.result_collision = [];	// 次回のdetectCollision呼び出しまでの間結果を保存するため
  }
  prepareVectors(numJoints, destinationSize) {
    // ******** destination variables ********
    this.controllerTfVec = new Float64Array(destinationSize); // controllerから受け取った目標位置姿勢Float64Arrayの4x4同次座標系サイズ16
    this.controllerJointVec = new Float64Array(numJoints); // controllerから受け取った目標ジョイントベクトル
    // 
    this.endLinkPoseVec = new Float64Array(destinationSize); // 現在の目標値
    this.jointRewinder = null; // TrapVelocGeneratorの配列
    this.joints = new Float64Array(numJoints); // joint position vector. size is 6,7 or 8
    this.prevJoints = new Float64Array(numJoints); // 前回のジョイントポジション
    this.velocities = new Float64Array(numJoints);
    this.logPrevJoints = new Float64Array(numJoints); // ログ出力用の比較対象前回ジョイントポジション
    // ******** joint limit variables ********
    this.jointUpperLimits = new Float64Array(numJoints).fill(1e10);
    this.jointLowerLimits = new Float64Array(numJoints).fill(-1e10);
    this.limitFlags = new Int32Array(numJoints).fill(0);
    // ******** args for WASM ********
    // this.jointVec = null; // on demandでmakeDoubleVectorGで生成する
    // this.endLinkPose = null; // on demandでmakeDoubleVectorGで生成する
  }
  setJointLimits(lowerLimits, upperLimits) {
    if ((Array.isArray(lowerLimits) || lowerLimits instanceof Float64Array) 
	&& lowerLimits.length !== this.joints.length) {
      console.error('setJointLimits: lowerLimits length mismatch');
      return;
    }
    if ((Array.isArray(upperLimits) || upperLimits instanceof Float64Array)
	&& upperLimits.length !== this.joints.length) {
      console.error('setJointLimits: upperLimits length mismatch');
      return;
    }
    this.jointLowerLimits.set(lowerLimits);
    this.jointUpperLimits.set(upperLimits);
  }
  prepareCmdVelGen(wasmObj, wasm=this.slrmModule) {
    this.cmdVelGen = wasmObj;
    this.slrmModule = wasm;

    // **** constants ****
    this.SLRM_STAT = {};
    const SLRM_STATUS = this.SLRM_STAT;
    SLRM_STATUS.OK = wasm.CmdVelGeneratorStatus.OK.value;
    SLRM_STATUS.ERROR = wasm.CmdVelGeneratorStatus.ERROR.value;
    SLRM_STATUS.END = wasm.CmdVelGeneratorStatus.END.value;
    SLRM_STATUS.SINGULARITY = wasm.CmdVelGeneratorStatus.SINGULARITY.value;
    SLRM_STATUS.REWIND = wasm.CmdVelGeneratorStatus.REWIND.value;
    this.statusName = {
      [SLRM_STATUS.OK]: 'OK',
      [SLRM_STATUS.ERROR]: 'ERROR',
      [SLRM_STATUS.END]: 'END',
      [SLRM_STATUS.SIMGILARITY]: 'SINGULARITY',
      [SLRM_STATUS.REWIND]: 'REWIND',
    };
    Object.freeze(this.SLRM_STAT);
    Object.freeze(this.statusName);

    this.jointVec = new wasm.DoubleVector();
    this.jointVec.resize(this.joints.length,0);
    this.endLinkPose = new wasm.DoubleVector();
    this.endLinkPose.resize(this.endLinkPoseVec.length,0);
    this.emptyEndLinkPose = new wasm.DoubleVector();
    this.emptyEndLinkPose.resize(0);
    this.limitFlagsWasm = new wasm.Int32Vector();
    this.limitFlagsWasm.resize(this.joints.length,0);
  }
  deleteSlrm() {
    if (this.jointVec) this.jointVec.delete();
    if (this.endLinkPose) this.endLinkPose.delete();
    if (this.emptyEndLinkPose) this.emptyEndLinkPose.delete();
    if (this.limitFlagsWasm) this.limitFlagsWasm.delete();
  }
  prepareGjkCd(wasmObj, wasm=this.cdModule) {
    this.gjkCd = wasmObj;
    this.cdModule = wasm;
    this.jointPosition = new wasm.DoubleVector();
    this.jointPosition.resize(this.joints.length,0);
    // this.jointVecと同じだがWASMモジュールが異なる
  }
  deleteGjkCd() {
    if (this.jointPosition) this.jointPosition.delete();
  }
  // ******** collision detection function ********
  detectCollisions(joints, result_collision) {
    if (!this.ignoreCollision && this.gjkCd) {
      copyArrayToWasmVec(joints, this.jointPosition); // , this.cdModule);
      this.gjkCd.calcFk(this.jointPosition);
      const resultPairs = this.gjkCd.testCollisionPairs();
      // struct UnsignedPair { unsigned int first, second; };
      // type of resultPairs is std::vector<UnsignedPair>
      //register_vector<in_house::UnsignedPair>("UnsignedPairVector")
      result_collision.length = 0; // clear previous results
      const count = resultPairs.size();
      for (let i=0; i<count; i++) {
	const pair = resultPairs.get(i);
	result_collision.push([pair.first, pair.second]);
      }
      resultPairs.delete();
      return count;
    }
    return 0;
  }
  //
  doJointMove(timeStep) {
    if (this.controllerJointVec &&
	this.controllerJointVec.length === this.joints.length) {
      let allReached = true;
      this.prevJoints.set(this.joints);
      for (let i=0; i<this.joints.length; i++) {
	let jointVel = this.jMoveGain *
	    (this.controllerJointVec[i] - this.joints[i]);
	if (jointVel < -this.jMoveVelocityLimit) {
	  jointVel = -this.jMoveVelocityLimit;
	} else if (jointVel > this.jMoveVelocityLimit) {
	  jointVel = this.jMoveVelocityLimit;
	}
	this.velocities[i] = jointVel;
	this.prevJoints[i] = this.joints[i];
	this.joints[i] = this.joints[i] + this.velocities[i] * timeStep;
	const diff = this.controllerJointVec[i] - this.joints[i];
	if (diff < -1e-2 || diff > 1e-2) {
	  allReached = false;
	}
      }
      if (allReached) {
	this.subState = sst.converged;
      }
      return true;
    } else {
      console.error('controllerJointVec is not set properly for joint move');
      return false;
    }
  }
  doRewind(timeStep) {
    const res = this.jointRewinder.map((der,i)=>
      der.calcNext(this.joints[i], this.velocities[i], timeStep));
    let allReached = true;
    this.prevJoints.set(this.joints);
    for (let i=0; i<this.joints.length; i++) {
      let diff = res[i].x - this.joints[i];
      this.joints[i] = res[i].x;
      this.velocities[i] = res[i].v;
      if (diff < -1e-2 || diff > 1e-2) {
	allReached = false;	// res[i].constrainedは使わない
      }
    }
    if (allReached) {
      this.subState = sst.converged;
    }
    const socket = bridge.socket;
    if (socket) { // デバッグ用出力
      const msg = {
	topic:'actuator1',
	javascriptStamp: Date.now(),
	header: { },
	position: [...this.joints],
	velocity: [...this.velocities],
	normalized: []
      }
      const binary = encode(msg);
      if (socket.readyState === WebSocket.OPEN) {
	socket.send(binary);
      } else {
	if (bridge.url) {
	  console.log('Not connected, queueing message');
	  bridge.messageQueue.push(msg);
	  if (!socket || socket.readyState === WebSocket.CLOSED) {
	    bridge.connect();
	  }
	}
      }
    }
    return allReached;
  }

  // ***** main function called in each loop *****
  step(timeStep) {
    if (this.subState === sst.dormant) return;
    if (!this.slrmModule) return;
    let noDestination = this.noDestination;
    let result_status_value = null;
    let result_other = null;
    if (!this.cmdVelGen || !this.joints) return;
    if (this.state === st.slrmReady &&
	(this.subState === sst.moving ||
	 this.subState === sst.jMoving ||
	 this.subState === sst.rewinding)) {
      if (this.subState === sst.moving && this.controllerTfVec &&
	  this.controllerTfVec.length === this.endLinkPoseVec.length) {
	this.endLinkPoseVec.set(this.controllerTfVec);
      } else if (this.subState === sst.jMoving) {
	if (this.doJointMove(timeStep) === true) {
	  // jMovingの場合、衝突検出を行う
	  if (this.detectCollisions(this.joints, this.result_collision) !== 0) {
	    this.joints.set(this.prevJoints); // 衝突したら前の状態に戻す
	    this.subState = sst.converged; // 衝突したら動作終了
	  }
	  // endLinkPoseVec = [];
	  noDestination = true; //現在値をゴールにしてcalcVelocityPQを1回実行する
	}
      }
    } else if (this.subState === sst.rewinding) {
      if (this.doRewind(timeStep) === true) {
	// endLinkPoseVec = [];
	noDestination = true;// 現在値をゴールにしてcalcVelocityPQを1回実行する
      }
    } else {
      noDestination = true; // 現在値をゴールにしてcalcVelocityPQを1回実行する
    }
    copyArrayToWasmVec(this.joints, this.jointVec); // , this.slrmModule);
    copyArrayToWasmVec(this.endLinkPoseVec, this.endLinkPose); // , this.slrmModule);
    let result = null;
    if (this.jointLimitKeepMoving) {
      copyArrayToWasmVec(this.limitFlags, this.limitFlagsWasm); // , this.slrmModule);
      result = this.cmdVelGen.calcVelocityPQ2(this.jointVec,
						    noDestination ?
						    this.emptyEndLinkPose :
						    this.endLinkPose,
						    this.limitFlagsWasm);
    } else {
      result = this.cmdVelGen.calcVelocityPQ(this.jointVec,
						   noDestination ?
						   this.emptyEndLinkPose :
						   this.endLinkPose);
    }
    this.noDestination = false; // reset
    if (this.subState === sst.moving) {
      // for (let i=0; i<velocities.length; i++) {
      // 	velocities[i] = result.joint_velocities.get(i);
      // 	// endLinkPose.lengthが0の場合、velocitiesは0が約束されている
      // }
      copyWasmVecToArray(result.joint_velocities, this.velocities, this.slrmModule);
    }
    result.joint_velocities.delete();
    result_status_value = result.status.value;
    result_other = result.other;
    const position = new Float64Array(3);
    const quaternion = new Float64Array(4);
    copyWasmVecToArray(result.position, position, this.slrmModule);
    copyWasmVecToArray(result.quaternion, quaternion, this.slrmModule);
    result.position.delete();
    result.quaternion.delete();
    // console.debug('status: ', result.status.value);
    if (this.subState === sst.rewinding &&
	result.status.value !== this.SLRM_STAT.END &&
	result.status.value !== this.SLRM_STAT.OK) {
      console.warn('CmdVelGenerator returned status other than END or OK during rewinding:', this.statusName[result.status.value]);
    }
    if (this.subState === sst.moving) {
      switch (result.status.value) {
      case this.SLRM_STAT.OK:
	this.prevJoints.set(this.joints);
	for (let i=0; i<this.joints.length; i++) {
	  this.joints[i] = this.joints[i] + this.velocities[i]* timeStep;
	}
	if (this.detectCollisions(this.joints, this.result_collision) !== 0) {
	  this.joints.set(this.prevJoints); // 衝突したら前の状態に戻す
	}
	break;
      case this.SLRM_STAT.END:
	// 目標位置に到達した場合の処理
	// cmdPoseExists = false; 
	this.subState = sst.converged;
	break;
      case this.SLRM_STAT.SIMGILARITY:
	// 現状のCmdVelGeneratorではこの状態は発生せずREWINDに変わる
	// cmdPoseExists = false; // cmdPoseが存在しない
	console.error('CmdVelGenerator returned SINGULARITY status');
	break;
      case this.SLRM_STAT.REWIND:
	this.joints.set(this.prevJoints); // 前の状態に戻す. 特異点に入る直前の状態になる
	// cmdPoseExists = false; // cmdPoseが存在しない
	break;
      case this.SLRM_STAT.ERROR:
	console.error('CmdVelGenerator returned ERROR status');
	break;
      default:
	console.error('Unknown status from CmdVelGenerator:', result.status.value);
	break;
      }
    }
    if (result_status_value !== null && result_other !== null) {
      // let limitFlag = Array(this.joints.length).fill(0);
      this.limitFlags.fill(0);
      if (!this.ignoreJointLimits) {
	let jointLimitExceed = false;
	for (let i=0; i<this.joints.length; i++) {
	  if (this.joints[i] >= this.jointUpperLimits[i]) {
	    this.limitFlags[i] = 1;
	    this.prevJoints[i] = this.jointUpperLimits[i]; // - 0.001;
	    jointLimitExceed = true;
	  }
	  if (this.joints[i] <= this.jointLowerLimits[i]) {
	    this.limitFlags[i] = -1;
	    this.prevJoints[i]  = this.jointLowerLimits[i]; // + 0.001;
	    jointLimitExceed = true;
	  }
	}
	if (jointLimitExceed) {
	  this.joints.set(this.prevJoints);
	  if (!this.jointLimitKeepMoving) {
	    this.subState = sst.converged; // ジョイントリミットに達したら動作終了
	  }
	}
      }
      self.postMessage({type: 'joints', joints: this.joints});
      self.postMessage({type: 'status', status: this.statusName[result_status_value],
			exact_solution: this.exactSolution,
			condition_number: result_other.condition_number,
			manipulability: result_other.manipulability,
			sensitivity_scale: result_other.sensitivity_scale,
			limit_flag: this.limitFlags,
			collisions: this.result_collision
		       });
      self.postMessage({type: 'pose',
			position: position,
			quaternion: quaternion,
		       },[position.buffer, quaternion.buffer]);
      this.counter ++;
      if (this.logInterval !== 0n && this.counter % this.logInterval === 0n) {
	if (// this.logPrevJoints !== null && this.joints !== null &&
	  this.logPrevJoints.length === this.joints.length) {
	  let max = 0;
	  for (let i=0; i<this.joints.length; i++) {
	    const diff = Math.abs(this.logPrevJoints[i] - this.joints[i]);
	    if (diff > max) { max = diff; }
	  }
	  if (max > 0.005) {
	    // ログ出力
	    console.log('counter:', this.counter,
			'status: ', this.statusName[result_status_value] ,
			' condition:' , result_other.condition_number.toFixed(2) ,
			' m:' , result_other.manipulability.toFixed(3) ,
			' k:' , result_other.sensitivity_scale.toFixed(3)
			+ '\n' +
			'limit flags: ' + this.limitFlag.join(', '));
	    //   console.debug('Worker: joints at ' + (counter / (60n*100n / BigInt(this.timeInterval))).toString() + ' minutes: ' + this.joints.map(v => (v*57.2958).toFixed(1)).join(', '));
	  }
	}
	this.logPrevJoints.set(this.joints); // ログ出力用の前回ジョイントポジションを更新 配列の複製不要
      }
    }
  }
}
export { IkCdCalc };
