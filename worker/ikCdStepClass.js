// Worker script for handling messages and performing calculations
// worker state definition
// import {TrapVelocGenerator} from './TrapVelocGenerator.js';
import { encode } from '@msgpack/msgpack';

import { st, sst, bridge } from './workerStateSingleton.js';
import { customLogger } from './customLogger.js';
globalThis.__customLogger = customLogger;
const ucl_logger = globalThis.__customLogger;
// const ucl_logger = customLogger;

import { RewindQueue} from './rewindQueue.js';


function copyArrayToWasmVec(jsArray, emVec) { // , wasmModule) {
  for (let i = 0; i < emVec.size(); ++i) {
    emVec.set(i, jsArray[i]);
  }
}
function copyWasmVecToArray(emVec, jsArray) { // , wasmModule) {
  for (let i = 0; i < jsArray.length; ++i) {
    jsArray[i] = emVec.get(i);
  }
}

class IkCdCalc {
  constructor (slrmModule, cdModule, cmdQueue) {
    // this.jointLimitKeepMoving = true;
    this.jointLimitKeepMoving = false;
    this.slrmModule = slrmModule; // SLRM WASM module
    this.cdModule = cdModule;     // Collision Detection WASM module
    this._cmdQueue = cmdQueue;	// move command queue
    this.state =  st.initializing; // worker state
    this.subState =  sst.dormant;  // slrm & joint mover/rewinder state
    this.timeInterval = 4; // time step for simulation in milliseconds
    this.postInterval = 33; // interval for posting messages to main thread in milliseconds
    this.postTimer = 0; // timer for posting messages to main thread
    this.logInterval = 0n/BigInt(this.timeInterval); // log interval in BigInt
    // ******** flags ********
    this.noDestination = true; // 目標位置姿勢が存在しないかどうか
    // this.newDestinationFlag = false; // 新しいdestinationが来たかどうか
    this.exactSolution = false; // singularity通過のための設定
    this.ignoreCollision = false; // 干渉判定を無視するかどうか
    this.ignoreJointLimits = false; // ジョイントリミットを無視するかどうか
    // ******** wasm objects (set by the onmessage handler) ********
    this.cmdVelGen = null;
    this.cdPort = null;
    this.sequence = 0;
    // ******** control parameters ********
    this.counter = 0n;
    this.jMoveGain = 10.0; // joint move command gain
    this.jMoveVelocityLimit = Math.PI/3.0; // joint move command velocity limit rad/s
    // ******** result variables ********
    // this.result_collision = [];	// 次回のdetectCollision呼び出しまでの間結果を保存するため
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
      ucl_logger?.error('setJointLimits: lowerLimits length mismatch');
      return;
    }
    if ((Array.isArray(upperLimits) || upperLimits instanceof Float64Array)
	&& upperLimits.length !== this.joints.length) {
      ucl_logger?.error('setJointLimits: upperLimits length mismatch');
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

  prepareRewindQueue() {
    ucl_logger?.debug('Preparing rewind queue for collision detection');
    this.rewindQueue = new RewindQueue(100, this.joints);
    this.setNewData = this.rewindQueue.getOnmessageHandler();
    // rewindQueueはonmessage handlerによって常に先頭は直近のrewind可能な
    // joint値になっているがmessageが来ない場合は更新されない
    // その場合はnewestResultも[]になったまま変わらない
  }

  prepareGjkCd(port) {
    ucl_logger?.debug('Preparing GJK Collision Detection with port:', port);
    this.cdPort = port;
    this.cdPort.onmessage = (event) => {
      switch (event.data.command) {
      case 'query_ab_id_response':
	this.abId = event.data.abId;
	ucl_logger?.log('Received abId from cdWorker:', this.abId);
	break;
      case 'collision_pairs':
	this.setNewData(event.data.sequence, event.data.rbIds);
	break;
      default:
	ucl_logger?.warn('Unknown command received in cdPort:',
			 event.data.command);
	break;
      }
    };
  }
  deleteGjkCd() {
  }

  // ******** collision detection function ********
  // cd_workerに干渉チェック対象の剛体の位置姿勢を送る
  sendLinkCoordsToCd(joints) {
    if (!this.ignoreCollision &&
	this.cdPort &&
	this.abId !== undefined) {
      const ptr = this.cmdVelGen.getJointValuesBufferPtr();
      const size = this.cmdVelGen.getJointValuesBufferSize();
      const jointsWasm = new Float64Array(this.slrmModule.HEAPF64.buffer,
					  ptr, size);
      jointsWasm.set(joints);
      this.cmdVelGen.calcFk0(); // this calls calcWTLinks() internally
      const srcPtr = this.cmdVelGen.getWTLinksBufferPtr();
      const srcSize = this.cmdVelGen.getWTLinksBufferSize();
      const linkCoord = new Float64Array(this.slrmModule.HEAPF64.buffer,
					 srcPtr, srcSize);
      const copiedLinkCoord = linkCoord.slice();
      this.rewindQueue.enqueue(joints);
      this.cdPort.postMessage({ command: 'rb_poses',
				abId: this.abId,
				sequence: this.sequence,
				poses: copiedLinkCoord
			      }, [copiedLinkCoord.buffer]);
      this.sequence++;
    }
  }

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
      ucl_logger?.error('controllerJointVec is not set properly for joint move');
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
	  ucl_logger?.log('Not connected, queueing message');
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
  step(timeDelta) { // timeDelta is in milliseconds
    this.postTimer += timeDelta;
    const timeStep = timeDelta / 1000; // convert to seconds
    if (this.subState === sst.dormant) return;
    if (!this.slrmModule) return;
    if (!this.rewindQueue) return; // joint初期値が無くprepareGjkCd()未実行
    // もしWASMに渡していないbase_coordがあれば渡す。calcFk0()の中でbase_coordを使う
    const global_base_coord = globalThis.base_coord;
    if (global_base_coord && global_base_coord.length === 16 &&
	global_base_coord[15] > 0 &&
	typeof this.cmdVelGen?.notifyWTBaseBufferUpdated === 'function') {
      const ptr = this.cmdVelGen.getWTBaseBufferPtr();
      const size = this.cmdVelGen.getWTBaseBufferSize();
      if (ptr && size) {
	const wTbaseArray = new Float64Array(this.slrmModule.HEAPF64.buffer, ptr, size);
	wTbaseArray.set(global_base_coord);
	this.cmdVelGen.notifyWTBaseBufferUpdated();
	global_base_coord[15] = -1; // 使用済、未定義にする
      }
    }
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
	  this.sendLinkCoordsToCd(this.joints);
	  // 大抵1回前のcd結果が入っている
	  if (this.rewindQueue.newestResult.length) {
	    // detect collision(s)
	    this.joints.set(this.rewindQueue.getRewindElement());
	    this.subState = sst.converged; // 衝突したら動作終了
	  }
	  // endLinkPoseVec = [];
	  noDestination = true; //現在値をゴールにしてcalcVelocityPQを1回実行
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
      // 現状のリミット状態をセットしてPQ2で計算
      copyArrayToWasmVec(this.limitFlags, this.limitFlagsWasm);
      result = this.cmdVelGen.calcVelocityPQ2(this.jointVec,
					      noDestination ?
					      this.emptyEndLinkPose :
					      this.endLinkPose,
					      this.limitFlagsWasm);
    } else {
      // 普通にjoint速度を計算
      result = this.cmdVelGen.calcVelocityPQ(this.jointVec,
					     noDestination ?
					     this.emptyEndLinkPose :
					     this.endLinkPose);
    }
    this.noDestination = false; // reset
    if (this.subState === sst.moving) {
      copyWasmVecToArray(result.joint_velocities, this.velocities);
    }
    // calcVelocityPQまたはcalcVelocityPQ2とペアとなるdelete呼び出し
    result.joint_velocities.delete();
    result_status_value = result.status.value;
    result_other = result.other;
    const position = new Float64Array(3);
    const quaternion = new Float64Array(4);
    copyWasmVecToArray(result.position, position); // this.slrmModule);
    copyWasmVecToArray(result.quaternion, quaternion); //this.slrmModule);
    result.position.delete();
    result.quaternion.delete();
    // ucl_logger?.debug('status: ', result.status.value);
    if (this.subState === sst.rewinding &&
	result.status.value !== this.SLRM_STAT.END &&
	result.status.value !== this.SLRM_STAT.OK) {
      ucl_logger?.warn('CmdVelGenerator returned status other than END or OK during rewinding:', this.statusName[result.status.value]);
    }
    if (this.subState === sst.moving) {
      switch (result.status.value) {
      case this.SLRM_STAT.OK:
	this.prevJoints.set(this.joints);
	for (let i=0; i<this.joints.length; i++) {
	  this.joints[i] = this.joints[i] + this.velocities[i]* timeStep;
	}
	this.sendLinkCoordsToCd(this.joints);
	if (this.rewindQueue.newestResult.length) {
	  this.joints.set(this.rewindQueue.getRewindElement());
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
	ucl_logger?.error('CmdVelGenerator returned SINGULARITY status');
	break;
      case this.SLRM_STAT.REWIND:
	this.joints.set(this.prevJoints); // 前の状態に戻す. 特異点に入る直前の状態になる
	// cmdPoseExists = false; // cmdPoseが存在しない
	break;
      case this.SLRM_STAT.ERROR:
	ucl_logger?.error('CmdVelGenerator returned ERROR status');
	break;
      default:
	ucl_logger?.error('Unknown status from CmdVelGenerator:', result.status.value);
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
	    this.joints[i] = this.prevJoints[i];
	    this.prevJoints[i] = this.jointUpperLimits[i]; // - 0.001;
	    jointLimitExceed = true;
	  }
	  if (this.joints[i] <= this.jointLowerLimits[i]) {
	    this.limitFlags[i] = -1;
	    this.joints[i] = this.prevJoints[i];
	    this.prevJoints[i]  = this.jointLowerLimits[i]; // + 0.001;
	    jointLimitExceed = true;
	  }
	}
	if (jointLimitExceed) {
	  if (!this.jointLimitKeepMoving) {
	    this.joints.set(this.prevJoints);
	    this.subState = sst.converged; // ジョイントリミットに達したら動作終了
	  }
	}
      }
      if (this.postTimer >= this.postInterval) {
	self.postMessage({type: 'joints', joints: this.joints});
	self.postMessage({type: 'status', status: this.statusName[result_status_value],
			  exact_solution: this.exactSolution,
			  condition_number: result_other.condition_number,
			  manipulability: result_other.manipulability,
			  sensitivity_scale: result_other.sensitivity_scale,
			  limit_flag: this.limitFlags,
			  collisions: this.rewindQueue.newestResult
			 });
	// newestResultは旧result_collisionと仕様が違うが多分reflectCollisionは
	// 働く
	self.postMessage({type: 'pose',
			  position: position,
			  quaternion: quaternion,
			 },[position.buffer, quaternion.buffer]);
	this.postTimer = 0;
      }
      if (this.subState === sst.converged) {
	// cmdQueueを確認して新しいコマンドがあれば開始する
	if (this._cmdQueue.length > 0) {
	  const cmd = this._cmdQueue.shift();
	  if (cmd.type === 'jMove') {
	    this.controllerJointVec.set(cmd.joints);
	    this.subState = sst.jMoving;
	  }
	}
      }
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
	    ucl_logger?.log('counter:', this.counter,
			'status: ', this.statusName[result_status_value] ,
			' condition:' , result_other.condition_number.toFixed(2) ,
			' m:' , result_other.manipulability.toFixed(3) ,
			' k:' , result_other.sensitivity_scale.toFixed(3)
			+ '\n' +
			'limit flags: ' + this.limitFlag.join(', '));
	    //   ucl_logger?.debug('Worker: joints at ' + (counter / (60n*100n / BigInt(this.timeInterval))).toString() + ' minutes: ' + this.joints.map(v => (v*57.2958).toFixed(1)).join(', '));
	  }
	}
	this.logPrevJoints.set(this.joints); // ログ出力用の前回ジョイントポジションを更新 配列の複製不要
      }
    }
  }
}
export { IkCdCalc };
