# ik-worker, cd-worker分離
## main
1. new Worker('ik-worker.js')
2. new MessageChannel()
3. onmessage: case 'ready':  
   1. post "set_port" channel.port1 to cd-worker
   2. post "set_port" channel.port2 to ik-worker
   3. post initMsg to ik-worker
   よってik-workerは、initMsgを受け取った時には既にport1を持っているはず

## ik-worker
`slrm_wasm`はそのまま無改造(link poseが取り出せるver.0.4.8以降)で使用する
現在jsから使用していないがWASMには`wTbase`関連は直接HEAPアクセスで既に実装すみ。
1. `set_port`メッセージを受けてcdWorker用portをselfに付ける
2. `init`で`linkShapes`を解析してTypedArrayにしてcdWorkerにpost

joints(ジョイント値)がIKで計算された後、衝突検出のために
リンクの位置姿勢(IKのWASMが計算)をGJK CDのWASMに渡しす。
過去のjointsから計算された衝突ペアの最新値を調べ、衝突していたら
衝突していない一番最近のjointsをrewindQueueからとりだし
(getRewindElememt)てそこまでrewindする
rewindQueueは初期化時に干渉していないjointsを1個目の値として入れてある
ため、getRewindElementは必ず干渉していないjointsを取り出すことができる
rewindQueueのnewestResultが[]かどうか調べて、[]ならは干渉無し

なかみ(step)は、detectCollisionsを、sendLinkCoordsToCdと
checkCollisionに分離する。sendはcd-workerにpostMessageし、
checkはchannelのonmessageハンドラーが更新している最新の干渉情報を
読み出すが、this.rewindQueueが情報を保持しているため
this.result_collisionへのコビーは不要であろう。必要になる直前まで
タイミングを後ろにずらすと極力新しいデータが取れる

`ik-worker`がselfのonmessageでメインから受けるコマンドには因果関係があり、
メインのpostMessage側で状態管理してpostしているが、ik-worker側でpromiseを
作りながら順序を整理して実行するように変更しても良い。


## cd-worker
`gjk_wasm`はposeのsetは(ver.0.4.8版と)おなじ。しかし、結果の取り出しを
HEAPから直接取り出すように変更する。結果はHEAPから配列かTypedArrayに
コピーしてik-workerにpostMessageで(TypedArrayの場合はtransferで)送る
`gjk_wasm`はconvex hullのデータ構造のJSとの受け渡しが全面変更になるので
その部分も書き直す必要がある


## robot-loader
`robot-loader`の`attach-to-another`は、DOM組み換え(a-frame組み換え)を
やめてTHREEの親子関係だけ組み替える実装を試す。これによりtickの呼び出し不安定解消と
ik-workerのライフサイクル管理のremoveの実装復活が期待できる。

# workerがonMessageで受け取る移動命令毎の動作

* `set_initial_joints`: これが呼ばれないとmain stateが`slrm_ready`に
  ならないため他の命令が無視される。substateを確認することなく、初期pose設定の
  ためsubstateを`moving`にする
* `destination`: この命令のときは`rewinding`や`jMoving`状態なら
  この`destination`命令を無視する(すぐに次が来ることが期待されているため)
* `set_joint_target`: この命令は、`rewinding`や`moving`なら
  `moveCommandQueue`に積む。step()の最後にsubstateが`converged`に
  なっていたらdequeueして`jMoving`stateに遷移させる
* `slow_rewind`: この命令の時は `moving`, `jMoving`の場合即座中断し
  `rewinding`に遷移する
* `set_end_effector`系命令: 引数parseしてno destination(一発屋)で
  一瞬`moving`にして明示的にstep()を呼び、即座に元のstateに戻る

# obsolete
0. import globals and class definition
1. slrm module factory await import
2. cd module factory await import
3. SlrmModule await generation
4. CdModule await generation
5. SlrmModule return value definition generation

6. construct ik loop control object with step func (loopObject constructor)

7. onmessage handler
   7.1 init
	   7.1.1 prepare joints, endLink vectors in loopObject(prepareVectors)
	   7.1.2 construct cmdVelGen obj for cal
	   7.1.3 set cmdVelGen to loopObject(prepareCmdVelGen)
		     loopObject(prepareGjkCd) if needed
	   7.1.4 set joint limits loopObject(setJointLimits)
8.0 mainLoop function definition
8.1 self.postMessage({type: 'ready'}); mainLoop

# postMessageとonmessageでRPCのように対応をとる方法
```
const worker = new Worker('worker.js');
const pendingRequests = new Map(); // ID と resolve 関数のマッピング

// Worker からの返信を受け取る共通ハンドラー
worker.onmessage = (event) => {
  const { id, result, error, type } = event.data;
  
  if (pendingRequests.has(id)) {
    const { resolve, reject } = pendingRequests.get(id);
    pendingRequests.delete(id); // 完了したら削除
    
    if (error) reject(error);
    else resolve(result);
  }
};

// Promise でラップした送信関数
function sendCommand(type, payload) {
  const id = crypto.randomUUID(); // ユニークなIDを発行
  
  return new Promise((resolve, reject) => {
    pendingRequests.set(id, { resolve, reject });
    worker.postMessage({ id, type, payload });
  });
}

// 使い方：await で結果を待てるようになる
async function run() {
  try {
    const result = await sendCommand('calculate', { value: 10 });
    console.log('結果:', result);
  } catch (err) {
    console.error('エラー:', err);
  }
}
```
