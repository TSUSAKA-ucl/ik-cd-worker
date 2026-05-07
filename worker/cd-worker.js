'use strict';
import { customLogger } from './customLogger.js';
globalThis.__customLogger = customLogger;
const ucl_logger = globalThis.__customLogger;
if (typeof console.warn === 'function')  ucl_logger.warn = console.warn;
if (typeof console.error === 'function')  ucl_logger.error = console.error;
if (typeof console.log === 'function')  ucl_logger.log = console.log;
// if (typeof console.debug === 'function')  ucl_logger.debug = console.debug;
//
//
const CdModuleFactory = await import('/wasm/cd_module.js');

const CdModule = await CdModuleFactory.default();
// cd-worker.jsは、link_shapes_interface.cppを使っていて、
// CdModuleFactory.default()の時点でWASM globalなgjkCdが一つ生成
// それを使っていく。js側から明示的なgjkCdは不要
ucl_logger?.debug('CdModule loaded successfully', CdModule);
if (!CdModule) {
  ucl_logger?.error('Failed to load CdModule');
  throw new Error('CdModule could not be loaded');
}
CdModule.setJsLogLevel(3); // 3: info level, 4: debug level

// abはchannelと1対1で対応させる。abIdをchannelのindexとして使う。
// 名前はchannelの反対側で管理するため不要。このworkerのコードは書き換え。
// selfでない個々のchannelからは名前は付いてこない
// remove_portしてもabIdの使い回しはしない
self.alive = true;
self.CdModule = CdModule;
self.channel = []; // abIdをindexとしてportを保存する配列
self.registered = []; // link_shapeを受け取り登録済のabIdのリスト。登録されていないabIdはfalse
self.logTiming = false; // logTimingがtrueのとき、計算時間計測の結果をucl_logger.debugに出す。falseのときは出さない
self.logCollisionPairs = false; // logCollisionPairsがtrueのとき、衝突ペアの内容をucl_logger.debugに出す。falseのときは出さない

function main() {
  ucl_logger?.debug('cd-worker started, waiting for messages...');
  self.onmessage = (event) => {
    const data = event.data;
    switch (data.type) {
    case 'add_port':
      ucl_logger?.debug('Received add_port message', data);
      self.channel.push(data.port);
      // onmessageハンドラーを付ける関数を呼ぶ
      attachOnMessageHandler(data.port);
      break;
    case 'remove_port':
      // port関係を消し、WASMへのメモリー開放も促す
      cleanupAb(data.port);
      break;
    case '**log_timing': {
      if (data.timing) {
	self.logTiming = data.timing;
	ucl_logger?.debug('Timing log level set to', self.logTiming);
      } else {
	self.logTiming = false;
	ucl_logger?.debug('Timing log level disabled');
      }
    }
      break;
    case '**log_collision': {
      if (data.logCollision) {
	self.logCollisionPairs = data.logCollision;
	ucl_logger?.debug('Collision pairs log level set to', self.logCollisionPairs);
      } else {
	self.logCollisionPairs = false;
	ucl_logger?.debug('Collision pairs log level disabled');
      }
    }
      break;
    case 'shutdown':
      self.channel.forEach(port => {
	const abId = portToId(port);
	if (typeof port.close === 'function' && abId !== null) {
	  cleanupAb(abId, false);
	}
      });
      self.alive = false;
      break;
    default:
      break;
    }
  };

  // ************************************************
  // collision detectionのループを開始する
  const cdModule = self.CdModule;
  self.postMessage({ type: 'cd_worker_ready' });
  ucl_logger?.debug('cd_worker_ready message posted');
  // test_collision_pairs2にかかる時間の平均とばらつきと最大最小を計る
  let totalTime = 0;
  let totalTimeSquared = 0;
  let maxTime = 0;
  let minTime = Infinity;
  let countForTiming = 0;
  // log出力制御用のループカウンタ。0のときだけlog出力
  let loopCount = 0;
  const loop = () => {
    newestSequence.forEach((seq, abId) => {
      if (typeof abId === 'number' &&
	  typeof seq === 'number' &&
	  typeof newestPoses[abId] !== 'undefined') {
	rbCoordsUpdated(abId, seq, newestPoses[abId]);
	// if (loopCount===0) {
	//   CdModule.setJsLogLevel(4);
	//   // ucl_logger?.log('rbCoordsUpdated for:',abId);
	//   // ucl_logger?.log(`     newestPoses[${abId}]:`,newestPoses[abId]);
	// } else {
	//   CdModule.setJsLogLevel(3);
	// }
      }
    });
    // rbCoords有無はWASM側で管理する。rbCoordsが有るabはWASM内で正の
    // seq番号を持つことにして、無いabはseqを負の値にする。これで、
    // WASM側でrbCoordsが更新されたabだけを衝突判定するようにでき、
    // rbCoordsUpdatedを最小限に抑えられる
    newestSequence.length = 0;
    newestPoses.length = 0;
    if (cdModule._all_link_pose_defined() !== 0) {
      // WASM側で衝突判定を行う。結果はWASM内のバッファに書き込まれる
      if (loopCount === 0) ucl_logger?.debug('Running collision detection... ..................');
      // test_collision_pairs2にかかる時間の平均とばらつきと最大最小を計る
      const startTime = performance.now();
      cdModule._test_collision_pairs2();
      const endTime = performance.now();
      if (loopCount === 0) ucl_logger?.debug('Collision detection completed');
      const elapsedTime = endTime - startTime;
      totalTime += elapsedTime;
      totalTimeSquared += elapsedTime * elapsedTime;
      maxTime = Math.max(maxTime, elapsedTime);
      minTime = Math.min(minTime, elapsedTime);
      countForTiming++;
      // 一旦、colliding abIds bufferは中止。全abIdを走査することとする
      // const collidingAbIdsPtr = cdModule.getCollidingAbIdsBufferPtr();
      // const collidingAbIdsSize = cdModule.getCollidingAbIdsBufferSize();
      // const collidingAbIds = new Int32Array(self.CdModule.HEAP32.buffer,
      // 					  collidingAbIdsPtr,
      // 					  collidingAbIdsSize);
      // // abはik-workerと一対一に対応していて、個々に固有のseq番号がある
      // // collidingAbIdsの中身はabIDとsequenceが交互にならんでいる

      const collisionPairsPtr = cdModule._get_collision_pairs_buffer_ptr_();
      const collisionPairsSize = cdModule._get_collision_pairs_buffer_size_();
      const collisionPairs = new Int32Array(self.CdModule.HEAP32.buffer,
					    collisionPairsPtr,
					    collisionPairsSize);
      // 干渉している全rbIdのペアの配列。中身はrbId(通し番号)が交互に並んでいる
      // デバッグ用にcollisionPairsの内容をログに出す。
      // ucl_logger?.debug('Collision pairs:', collisionPairs);
      if (loopCount === 0 && collisionPairsSize > 0) {
	// const collisionPairsArray = Array.from(collisionPairs.slice(0, collisionPairsSize));
	const collisionPairsArray = Array.from(collisionPairs).reduce((acc, val, index) => {
	  if (index%2 === 0) acc.push([val]);
	  else acc[acc.length-1].push(val);
	  return acc;
	} , []);
	ucl_logger?.debug('Collision pairs array:', collisionPairsArray);
      }
      // ab毎に必要なrbIdを抽出して、abIdとsequenceとともにik-workerに送る
      // for (let i = 0; i < collidingAbIdsSize; i += 2) {
      //   const abId = collidingAbIds[i];
      for (let abId = 0; abId < cdModule._num_ab_objects(); abId++) {
	const rbIdOffsetMin = cdModule._query_rbId_offset(abId);
	const rbIdOffsetMax = cdModule._query_rbId_end(abId);
	const abCollisionRbIds = [];
	for (let j = 0; j < collisionPairsSize; j++) {
	  const rbId1 = collisionPairs[j];
	  if (rbIdOffsetMin <= rbId1 && rbId1 < rbIdOffsetMax) {
	    // ここでrbIdをab内の相対的な0オリジンのIDに変換し旧版と互換にする
	    abCollisionRbIds.push(rbId1 - rbIdOffsetMin);
	  }
	}
	// abStopDependencies走査もここで行う。abStopDependenciesはabIdをキーにして、そこから衝突判定により動作を止めるべきabIdの配列を引けるようになっている。abCollisionRbIdsに、abStopDependenciesで指定されたab内のrbId(負の数か範囲外の数)も追加する
	const stopAbIds = abStopDependencies[abId] || [];
	stopAbIds.forEach(stopAbId => {
	  const stopRbIdOffsetMin = cdModule._query_rbId_offset(stopAbId);
	  const stopRbIdOffsetMax = cdModule._query_rbId_end(stopAbId);
	  for (let j = 0; j < collisionPairsSize; j++) {
	    const rbId1 = collisionPairs[j];
	    if (stopRbIdOffsetMin <= rbId1 && rbId1 < stopRbIdOffsetMax) {
	      abCollisionRbIds.push(rbId1 - rbIdOffsetMin);
	    }
	  }
	});
	if (loopCount === 0 && abCollisionRbIds.length > 0 &&
	    self.logCollisionPairs) {
	  ucl_logger?.log(`Posting collision pairs for abId ${abId}:`, abCollisionRbIds);
	}
	self.channel[abId].postMessage({ command: 'collision_pairs',
					 sequence: cdModule._query_ab_sequence(abId),
					 data: abCollisionRbIds});
      }
    } else {
      if (loopCount === 0) {
	ucl_logger?.warn('Not all link poses defined yet, skipping collision detection');
      }
    }
    if (loopCount === 0 && countForTiming > 0) {
      if (self.logTiming) {
	const avgTime = totalTime / countForTiming;
	const variance = (totalTimeSquared / countForTiming) - (avgTime * avgTime);
	const stdDev = Math.sqrt(variance);
	ucl_logger?.log('Collision detection timing: ',
			`avg=${avgTime.toFixed(2)}ms, `,
			`stdDev=${stdDev.toFixed(2)}ms, `,
			`min=${minTime.toFixed(2)}ms, `,
			`max=${maxTime.toFixed(2)}ms`);
      }
    }
    if (++loopCount >= 2000) loopCount = 0;
    if (self.alive) setTimeout(loop, 4);
  };
  loop();
}

// link_shapesコマンドを受け取ったportのみregisteredになる
function portToId(port, add=false) {
  const abId = self.channel.indexOf(port);
  if (abId >= 0) {
    if (add) {
      if (self.channel[abId] === port) {
	self.registered[abId] = true;
	ucl_logger?.log('channel registered:', self.channel);
	return abId;
      } else {
	ucl_logger?.error('Port already exists at index but does not match the given port. This should not happen. abId:', abId);
	return null;
      }
    } else {
      if (self.registered[abId]===true) {
	return abId;
      } else {
	ucl_logger?.warn('Port found in channel but has no shapes yet. abId:', abId);
	return null;
      }
    }
  } else {
    if (add) {
      self.channel.push(port);
      const newAbId = self.channel.length - 1;
      self.registered[newAbId] = true;
      ucl_logger?.log('channel added and registered:', self.channel);
      return newAbId;
    } else {
      return null;
    }
  }
}

function cleanupAb(abId, memory=true) {
  if (self.channel[abId]) {
    // 対応するportをchannelから削除し、portを閉じ、onmessageハンドラーを外す
    if (typeof self.channel[abId].close === 'function') {
      self.channel[abId].postMessage = () => {};
      self.channel[abId].onmessage = null;
      self.channel[abId].close();
      // delete self.channel[abId];
    }
    if (memory) {
      // WASMのg_ab_objects_からabIdに対応するデータを削除する。
      // self.CdModule.ab_remove(abId);
    }
  }
}

const abStopDependencies = [];
// onmessageハンドラーをportに取り付ける関数。newestSequenceとnewestPosesは、座標更新のコマンドを受け取ったときに、abIdごとに最新のseq番号と座標を保存しておくためのオブジェクト。
const newestSequence = [];
const newestPoses = [];
function attachOnMessageHandler(port) {
  ucl_logger?.debug('Attaching onmessage handler to port', port);
  port.onmessage = (event) => {
    switch (event.data.command) {
    case 'link_shapes': {
      // これはcallRpcなのでuuidを付けて返す
      const abId = portToId(port, true);
      // self.channel[abId] = port;
      ucl_logger?.debug('Received link_shapes command. abLayer:', event.data.shapes.abLayer);
      ucl_logger?.debug('Received link_shapes command. uuid:', event.data.uuid);
      ucl_logger?.log('Registered abId',abId,'for',event.data.name);
      registerLinkShapes(abId, event.data.shapes);
      ucl_logger?.log('Finished registeration of link shapes for abId', abId);
      port.postMessage({ command: 'link_shapes_response',
			 uuid: event.data.uuid,
			 abId: abId});
    }
      break;
      // 当面、link_shapesコマンドのみで、
      // 座標更新・ab削除・sa層のつけ外しのコマンドは後ほど追加していく予定
    case 'ignore_pairs': {
      // これはcallRpcなのでuuidを付けて返す
      let result = -1;
      // cd-workerは、abId:linkをrb通し番号に変換して、ignore pairsをメンテする。
      // test pairs(all)を生成した(どこで??)後にignore pairsを取り除く
      //
      // 送られてくるignorePairsは以下のとおり。
	  // const ignorePairs = new Int32Array(main.ignorePairs.length * 4);
	  // main.ignorePairs.forEach((pair, index) => {
	  //   ignorePairs[index*4] = self.abId; // my_abId
	  //   ignorePairs[index*4 + 1] = pair.myLink;
	  //   ignorePairs[index*4 + 2] = pair.otherAbId; // other_abId
	  //   ignorePairs[index*4 + 3] = pair.otherLink;
	  // } );
      // linkはab内の相対的なID。このままWASMに渡して、WASM内でrb通し番号に変換
      // EMSCRIPTEN_KEEPALIVE uintptr_t exception_pairs_alloc(uint32_t length) {
      // EMSCRIPTEN_KEEPALIVE void add_exception_pairs() // add
      //if (event.data.ignorePairs && Array.isArray(event.data.ignorePairs)) {
      // event.data.ignorePairsがInt32Arrayであることを期待しているが、念のためチェックする
      if (!event.data.ignorePairs || !(event.data.ignorePairs instanceof Int32Array) ||
	  event.data.ignorePairs.length % 4 !== 0) {
	ucl_logger?.error('Invalid ignorePairs length:', event.data.ignorePairs.length);
      } else {
	ucl_logger?.debug('$$$$$$$$$$ Received ignore_pairs command with', event.data.ignorePairs.length / 4, 'pairs. Processing...'); // 4で割ってペア数を表示
	const cdModule = self.CdModule;
	const exceptionPairsPtr = cdModule._exception_pairs_alloc(event.data.ignorePairs.length);
	// 本当はWASMでvector resizeが成功しているかどうかを返してもらうべきだが、
	// ここではとりあえず成功している前提で進める
	const exceptionPairsArray = new Int32Array(self.CdModule.HEAP32.buffer,
						   exceptionPairsPtr,
						   event.data.ignorePairs.length);
	exceptionPairsArray.set(event.data.ignorePairs);
	cdModule._add_exception_pairs(); // 登録完了したらexceptionPairsPtrのvectorはshrink_to_fitされる
	// 登録後のignorePairsは、WASM側で管理されるため、ここでは特に何もしない
	ucl_logger?.debug('$$$$$ Ignore pairs registered in WASM module. Reconstructing test pairs...');
	// for (let abId = 0; abId < cdModule._num_ab_objects(); abId++) {
	//   ucl_logger?.debug('    abId:',abId,
	// 		    'rbMin:',cdModule._query_rbId_offset(abId),
	// 		    'rbMax:',cdModule._query_rbId_end(abId));
	// }
	cdModule._reconstruct_test_pairs(0);
	result = event.data.ignorePairs.length / 4; // 登録したペアの数を返す
      }
      port.postMessage({ command: 'ignore_pairs_response',
			 uuid: event.data.uuid,
			 result: result});
    }
      break;
    case 'stop_dependency': {
      // abIdをキーにして、そこから衝突判定により動作を止めるべきabId
      // の配列を引けるようにする。これで、あるabが衝突したときに、そ
      // のabだけでなく、関連するabもまとめて動作を止めることができる
      // ようになる。event.data.stopAbIdは、動作を止めるべきabId一個。
      // abStopDependenciesは、abIdをキーにして、そこから衝突判定によ
      // り動作を止めるべきabIdの配列を引けるようになっている。
      // これはcallRpcなのでuuidを付けて返す
      const abId = portToId(port);
      if (typeof abId === 'number') {
	ucl_logger?.debug(`Received stop_dependencies command for abId ${abId} with stopAbId:`, event.data.stopAbId);
	if (!abStopDependencies[abId]) { abStopDependencies[abId] = []; }
	// abStopDependenciesにevent.data.stopAbIdの値が無ければ追記する。重複は避ける。
	if (!abStopDependencies[abId].includes(event.data.stopAbId)) {
	  abStopDependencies[abId].push(event.data.stopAbId);
	}
	// さらにevent.data.stopAbIdの依存先があればその先も再帰的にす
	// べてabStopDependencies[abId]に追加する。これで、依存の連鎖も
	// 考慮できるようになる。
	const reconstructDependencies = (stopAbId) => {
	  if (abStopDependencies[stopAbId]) {
	    abStopDependencies[stopAbId].forEach(dependentAbId => {
	      if (!abStopDependencies[abId].includes(dependentAbId)) {
		abStopDependencies[abId].push(dependentAbId);
		reconstructDependencies(dependentAbId);
	      }
	    } );
	  }
	};
	reconstructDependencies(event.data.stopAbId);
	ucl_logger?.log(`## Updated stop dependencies for abId ${abId}:`, abStopDependencies[abId]);
	port.postMessage({ command: 'stop_dependencies_response',
			   uuid: event.data.uuid,
			   result: 'ok'});
      }
    }
      break;
    case 'query_ab_id': {
      // これはcallRpcなのでuuidを付けて返す
      const abId = portToId(port);
      port.postMessage({ command: 'query_ab_id_response',
			 uuid: event.data.uuid,
			 abId: abId});
    }
      break;
    case 'rb_poses': {
      // console.warn('Received rb_poses command. abId:', portToId(port),
      // 		   'sequence:', event.data.sequence, 'poses:', event.data.poses);
      // これはcallRpcではない。最新のseq番号と座標を保存しておくだけ
      const abId = portToId(port);
      if (typeof abId === 'number') { // 定義済のabIdは必ず数値だが、念のため
	newestSequence[abId] = event.data.sequence;
	newestPoses[abId] = event.data.poses;
      }
    }
      break;
    default:
      break;
    }
  };
}

// WASMにlink shapeを登録する関数。articulated body idと
// link shapeのデータを受け取る。
function registerLinkShapes(abId, packedData) {
  const cdModule = self.CdModule;
  // WASMにバッファを確保させそのアドレスをもらう
  // alloc, freeはextern "C"関数の直接呼び出し
  // length+1の+1は下のlayerの最後のindexを入れる場所。
  const packed = packedData;
  // ucl_logger?.log('registering packed data:',packed);
  const abPointer = cdModule._ab_alloc(packed.abLayer.length+1);
  const abLayer = new Int32Array(cdModule.HEAP32.buffer,
				 abPointer,
				 packed.abLayer.length+1);
  abLayer.set(packed.abLayer);
  abLayer[packed.abLayer.length] = packed.rbLayer.length; // abLayerの最後のindexにrbLayerの長さを入れる。これで、WASM側で最後のrbの長さを知ることができる
  const rbPointer = cdModule._rb_alloc(packed.rbLayer.length+1);
  const rbLayer = new Int32Array(cdModule.HEAP32.buffer,
				 rbPointer,
				 packed.rbLayer.length+1);
  rbLayer.set(packed.rbLayer);
  rbLayer[packed.rbLayer.length] = packed.saLayer.length;
  const saPointer = cdModule._sa_alloc(packed.saLayer.length+1);
  const saLayer = new Int32Array(cdModule.HEAP32.buffer,
				 saPointer,
				 packed.saLayer.length+1);
  saLayer.set(packed.saLayer);
  saLayer[packed.saLayer.length] = packed.vertices.length;
  // convex hullの頂点座標はfloat64で渡すためdoubleのバッファも確保してもらう
  const chPointer = cdModule._vertex_alloc(packed.vertices.length);
  const chLayer = new Float64Array(cdModule.HEAPF64.buffer,
				   chPointer,
				   packed.vertices.length);
  chLayer.set(packed.vertices);
  // base座標系はab登録時には特に渡さない。各rbのposeを渡す時にその値に含ませる
  // abId番のabとしてWASMに登録
  cdModule._add_link_shape2(abId);
  // addLinkShape2したらallocしたバッファは不要になるためfreeする。
  cdModule._ab_free();
  cdModule._rb_free();
  cdModule._sa_free();
  cdModule._vertex_free();
  // 以上で、WASMのglobal(g_ab_objects_にリンク形状構造のvectorができ
  // g_link_shapes_にCD用にフラット化したデータもできる
  for (const pair of packed.testPairs) {
    // ucl_logger?.debug('Registering test pair', pair, 'for abId', abId);
    cdModule._add_ab_test_pair(abId, pair[0],pair[1]);
  }
  ucl_logger?.debug("call cdModule._reconstruct_test_pairs(1);");
  cdModule._reconstruct_test_pairs(1);
}

// WASMにrbの座標が更新されたことを通知する関数。articulated body idと
// rbの座標の配列を受け取る。座標は、ik-workerのcalcFk0でworld座標系ベース
// の座標値を計算済。cd-workerはab毎のbase座標系の概念は無い
function rbCoordsUpdated(abId, sequence, poses) {
  const gjkCd = self.CdModule;
  // WASMのTypedArrayに座標をコピーして、更新を通知する
  // サイズはaddLinkShape2でWASM側に通知済で固定している
  const srcSize = poses.length;
  const destPtr = gjkCd._get_wTlinks_buffer_ptr2(abId);
  if (gjkCd._get_wTlinks_buffer_size2(abId) !== srcSize) {
    ucl_logger?.error('GJK CD buffer size mismatch: expected', srcSize,
		      'for abId', abId,
		      'but got', gjkCd._get_wTlinks_buffer_size2(abId));
    
    return;
  }
  const destArray = new Float64Array(self.CdModule.HEAPF64.buffer,
				     destPtr, srcSize);
  // console.warn('type of poses is', poses.constructor.name, 'length:', poses.length);
  destArray.set(poses);
  gjkCd._notify_link_coords_updated2(abId, sequence);
}

main();
// ======================
//  cdModule._test_collision_pairs2();
//  const collidingAbIdsPtr = cdModule._get_colliding_abIds_buffer_ptr();
//  const collidingAbIdsSize = cdModule._get_colliding_abIds_buffer_size_();
//  const collisionPairsPtr = cdModule._get_collision_pairs_buffer_ptr_();
//  const collisionPairsSize = cdModule._get_collision_pairs_buffer_size_();
