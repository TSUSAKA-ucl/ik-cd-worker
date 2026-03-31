# cdWorker.jsの説明コメント

## cd-worker設計方針
オブジェクト定義の階層

 * `ab`: articulated body: body内の全rigid bodiesの変換の配列をもらう。
   cd-workerがmainから受け取るmessage channelのportに一対一対応し、気持ちは
   一度に動かして衝突判定する(ばらばらに動かして衝突判定はしない)グループ
 * `rb`: rigid body: この要素の全sub assembliesは同一座標系に乗り相対的
   に動かない。sub assembly単位で追加・取り外しの可能性がある。
   articulated body内で通し番号がID
 * `sa`: sub assembly: プログラムコードにおける、登録削除の単位。
   rigid body内で通し番号だが当面は0のみ
 * `ch`: convex hull: 衝突判定の都合上sub assemblyを事前に分解しておく
   convex hull。全頂点の座標値を持つ

 WASMは、登録されたarticulated body単位でrigid bodiesの座標系の更新が
 あったことを`gjkCd.notifyLinkCoordsUpdated()`で知らされると内部モデル
 のconvex hullのvertexの座標を更新する。
 `testCollisionPairs()`で、必要なconvex hull間の衝突判定を行いrigid
 bodyの衝突ペアを返す。現状BVHを行っていないが、将来的にはconvex hull
 に対応するBVH作成用のデータを`addLinkShape()`に追加して、
 `testCollisionPairs()`でBVHを使った衝突判定も行う予定。

 0. `gjkCd = new CdModule.CollisionDetection()`でWASMインスタンスを生成する
 1. メインチャネルから、各motion workerからのチャネルのポートを受け取る
 2. 各motion workerからのチャネルのポートを配列に保存し、そのindexが
	articulated bodyのidになる
 3. チャネルからlinkShapesを受け取りrigid bodyの数だけ、convexWasmに変換
	して`gjkCd.addLinkShape()`する。このとき先頭のindexと
	articulated bodyのidは対応させる。

## cd-worker-main-loopの処理内容とWASMとのやりとり
 最速4msec周期で、gjkCd WASMモジュールで`testCollisionPairs2()`を呼び、
 衝突ペアをabID毎に分割して各motion workerに送るループ

`testCollisionPairs2()`は重く不定時間なので`setInterval`でなく、
`setTimeout`で呼び出し、呼び終わったらすぐ次を呼ぶ形(`setTimeout(loop,4)`)に
する  

WASMで、ab毎に、関係するrbが含まれる衝突ペアを分類して、HEAPにセットする  
rbは、全abで通し番号のIDを持っているが、ab毎に固まって範囲が決まっている  
基本的に一つのab内でのrbの数は変化しないが面倒なのでサイズもWASMから取得可能とする  
WASMにqueryすると各abに対応するrb番号のオフセット(最初の値)が得られる  
`testCollisionPairs2`の結果は、rbIDが2個づつ並んだ配列がab毎に用意される  
またcollisionが発生したrbを含むabのIDの配列も用意される  
`gjkCd.getCollidingAbIdsBufferPtr()`で、衝突が発生したabのIDの配列の
ポインタが得られる  
`gjkCd.getCollidingAbIdsBufferSize()`で、その配列のサイズが得られる  
`gjkCd.queryRbIdOffset(abId)`  
`gjkCd.queryRbLength(abId)`  
`gjkCd.getCollisionPairsBufferPtr(abId)`  
`gjkCd.getCollisionPairsBufferSize(abId)`  
`const collisionPairsArray = new Int32Array(....)`  
この配列に含まれるrbIdのうちオフセット範囲に収まっているもの(本abIdの
もの)の一覧を抽出して、abIdに対応するportにpostMessageする  
