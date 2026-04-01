'use strict';
import { customLogger } from './customLogger.js';
globalThis.__customLogger = customLogger;
const ucl_logger = globalThis.__customLogger;
if (typeof console.warn === 'function')
  ucl_logger.warn = console.warn
if (typeof console.error === 'function')
  ucl_logger.error = console.error
//
//
const CdModuleFactory = await import('/wasm/cd_module.js');
const CdModule = await CdModuleFactory.default();
if (!CdModule) {
  ucl_logger?.error('Failed to load CdModule');
  throw new Error('CdModule could not be loaded');
}
CdModule.setJsLogLevel(2); // 3: info level, 4: debug level

// abはchannelと1対1で対応させる。abIdをchannelのindexとして使う。
// 名前はchannelの反対側で管理するため不要。このworkerのコードは書き換え。
// selfでない個々のchannelからは名前は付いてこない
// remove_portしてもabIdの使い回しはしない
self.alive = true;
self.gjkCd = new CdModule.cd_constructor();
self.CdModule = CdModule;
self.channel = [];


function main() {
  self.onmessage = async (event) => {
    const data = event.data;
    switch (data.type) {
    case 'add_port':
      self.channel.push(data.port);
      // onmessageハンドラーを付ける関数を呼ぶ
      attachOnMessageHandler(data.port);
      break;
    case 'remove_port':
      // port関係を消し、WASMへのメモリー開放も促す
      cleanupAb(data.port);
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
  const gjkCd = self.gjkCd;
  const loop = () => {
    newestSequence.forEach((seq, abId) => {
      if (typeof seq === 'number' &&
	  typeof newestPoses[abId] !== 'undefined') {
	rbCoordsUpdated(abId, seq, newestPoses[abId]);
      }
    });
    // rbCoords有無はWASM側で管理する。rbCoordsが有るabはWASM内で正の
    // seq番号を持つことにして、無いabはseqを負の値にする。これで、
    // WASM側でrbCoordsが更新されたabだけを衝突判定するようにでき、
    // rbCoordsUpdatedを最小限に抑えられる
    newestSequence.length = 0;
    newestPoses.length = 0;
    // WASM側で衝突判定を行う。結果はWASM内のバッファに書き込まれる
    gjkCd.testCollisionPairs2();
    const collidingAbIdsPtr = gjkCd.getCollidingAbIdsBufferPtr();
    const collidingAbIdsSize = gjkCd.getCollidingAbIdsBufferSize();
    const collidingAbIds = new Int32Array(self.CdModule.HEAP32.buffer,
					  collidingAbIdsPtr,
					  collidingAbIdsSize);
    // abはik-workerと一対一に対応していて、個々に固有のseq番号がある
    // collidingAbIdsの中身はabIDとsequenceが交互にならんでいる

    const collisionPairsPtr = gjkCd.getCollisionPairsBufferPtr();
    const collisionPairsSize = gjkCd.getCollisionPairsBufferSize();
    const collisionPairs = new Int32Array(self.CdModule.HEAP32.buffer,
					  collisionPairsPtr,
					  collisionPairsSize);
    // 干渉している全rbIdのペアの配列。中身はrbId(通し番号)が交互に並んでいる

    // ab毎に必要なrbIdを抽出して、abIdとsequenceとともにik-workerに送る
    for (let i = 0; i < collidingAbIdsSize; i += 2) {
      const abId = collidingAbIds[i];
      const rbIdOffsetMin = gjkCd.queryRbIdOffset(abId);
      const rbIdOffsetMax = rbIdOffsetMin + gjkCd.queryRbLength(abId);
      const abCollisionRbIds = [];
      for (let j = 0; j < collisionPairsSize; j++) {
	const rbId1 = collisionPairs[j];
	if (rbIdOffsetMin <= rbId1 && rbId1 < rbIdOffsetMax) {
	  // ここでrbIdをab内の相対的な0オリジンのIDに変換し旧版と互換にする
	  abCollisionRbIds.push(rbId1 - rbIdOffsetMin);
	}
      }
      self.channel[abId].postMessage({ command: 'collision_pairs',
				       sequence: collidingAbIds[i + 1],
				       rbIds: abCollisionRbIds});
    }
    if (self.alive) setTimeout(loop, 4);
  };
  loop();
}

function portToId(port, add=false) {
  const abId = self.channel.indexOf(port);
  if (abId === -1) {
    if (add) {
      self.channel.push(port);
      return self.channel.length - 1;
    } else {
      return null;
    }
  }
  return abId;
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

// onmessageハンドラーをportに取り付ける関数。newestSequenceとnewestPosesは、座標更新のコマンドを受け取ったときに、abIdごとに最新のseq番号と座標を保存しておくためのオブジェクト。
const newestSequence = [];
const newestPoses = [];
function attachOnMessageHandler(port) {
  port.onmessage = async (event) => {
    switch (event.data.command) {
    case 'link_shapes': {
      const abId = portToId(port, true);
      // self.channel[abId] = port;
      registerLinkShapes(abId, event.data.packedData);
      port.postMessage({ command: 'query_ab_id_response',
			 abId: abId});
    }
      break;
      // 当面、link_shapesコマンドのみで、
      // 座標更新・ab削除・sa層のつけ外しのコマンドは後ほど追加していく予定
    case 'query_ab_id': {
      const abId = portToId(port);
      port.postMessage({ command: 'query_ab_id_response',
			 abId: abId});
    }
      break;
    case 'rb_poses': {
      const abId = portToId(port);
      if (typeof abId === 'number') { // abIdは必ず数値だが、念のため
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
  // WASMにバッファを確保させそのアドレスをもらう
  // alloc, freeはextern "C"関数の直接呼び出し
  // length+1の+1は下のlayerの最後のindexを入れる場所。
  const packed = packedData;
  const abPointer = self.CdModule._ab_alloc(packed.abLayer.length+1);
  const abLayer = new Int32Array(self.CdModule.HEAP32.buffer,
				 abPointer,
				 packed.abLayer.length+1);
  abLayer.set(packed.abLayer);
  abLayer[packed.abLayer.length] = packed.rbLayer.length; // abLayerの最後のindexにrbLayerの長さを入れる。これで、WASM側で最後のrbの長さを知ることができる
  const rbPointer = self.CdModule._rb_alloc(packed.rbLayer.length+1);
  const rbLayer = new Int32Array(self.CdModule.HEAP32.buffer,
				 rbPointer,
				 packed.rbLayer.length+1);
  rbLayer.set(packed.rbLayer);
  rbLayer[packed.rbLayer.length] = packed.saLayer.length;
  const saPointer = self.CdModule._sa_alloc(packed.saLayer.length+1);
  const saLayer = new Int32Array(self.CdModule.HEAP32.buffer,
				 saPointer,
				 packed.saLayer.length+1);
  saLayer.set(packed.saLayer);
  saLayer[packed.saLayer.length] = packed.vertices.length;
  // convex hullの頂点座標はfloat64で渡すためdoubleのバッファも確保してもらう
  const chPointer = self.CdModule._vertex_alloc(packed.vertices.length);
  const chLayer = new Float64Array(self.CdModule.HEAPF64.buffer,
				   chPointer,
				   packed.vertices.length);
  chLayer.set(packed.vertices);
  // base座標系はab登録時には特に渡さない。各rbのposeを渡す時にその値に含ませる
  // abId番のabとしてWASMに登録
  self.CdModule._addLinkShape2(abId);
  // addLinkShape2したらallocしたバッファは不要になるためfreeする。
  self.CdModule._ab_free();
  self.CdModule._rb_free();
  self.CdModule._sa_free();
  self.CdModule._vertex_free();
}

// WASMにrbの座標が更新されたことを通知する関数。articulated body idと
// rbの座標の配列を受け取る。座標は、ab登録時にはbase座標系を原点とした値で渡し、
// WASM側でabのbase座標系を考慮してワールド座標系に変換する。
function rbCoordsUpdated(abId, sequence, poses) {
  // WASMのTypedArrayに座標をコピーして、更新を通知する
  // サイズはaddLinkShape2でWASM側に通知済で固定している
  const srcSize = poses.length;
  const destPtr = self.gjkCd.getWTLinksBufferPtr2(abId);
  if (self.gjkCd.getWTLinksBufferSize2(abId) !== srcSize) {
    ucl_logger?.error('GJK CD buffer size mismatch: expected', srcSize,
		      'but got', self.gjkCd.getWTLinksBufferSize2(abId));
    return;
  }
  const destArray = new Float64Array(self.CdModule.HEAPF64.buffer,
				     destPtr, srcSize);
  destArray.set(poses);
  self.gjkCd.notifyLinkCoordsUpdated2(abId, sequence);
}

main();
