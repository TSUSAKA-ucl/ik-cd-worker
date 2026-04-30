# gjk_wasmインターフェース

## `gjk_worker`の`in_house_cd`の解説
* `Shape`はconvex hull 1個に対応する型
* `linkl_shapes_`変数は、`std::vector<std::vector<Shape>> link_shapes_`
* `std::vector<Eigen::Isometry3d> base_t_link_;`

## WASM側で準備・メンテする配列
### shape data入力用
#### ab table
全abのテーブルの配列。中身はrb tableのポインタ
動作確認版ではあまり意味はない。length of ab table === 1のため
#### rb table
全rbのテーブルの配列。中身はsa tableへのindexで、動作確認版では0,1,2,..,n
#### sa table
全saのテーブル。convex hull tableへのindex
#### ch table
全convex hullのテーブル

### 内部データ
入力データより `std::vector<std::vector<Shape>> link_shapes_` を作る
ab毎に作る気持ち。動作確認版ではabIdは0と決まっているので`XXX2`は旧版を流用できる



## 新API
### ab毎のリンク目標位置姿勢設定関係
```
const destPtr = self.gjkCd.getWTLinksBufferPtr2(abId);
if (self.gjkCd.getWTLinksBufferSize2(abId) !== srcSize) {}
self.gjkCd.notifyLinkCoordsUpdated2(abId, sequence);
```
### 干渉チェック計算。結果取り出し関係
```
gjkCd.testCollisionPairs2();
```
### 干渉しているabの表(seq含む)取り出し用
```
const collidingAbIdsPtr = gjkCd.getCollidingAbIdsBufferPtr();
const collidingAbIdsSize = gjkCd.getCollidingAbIdsBufferSize();
```
### 干渉している全rbIdのペアの配列の取り出し用
```
const collisionPairsPtr = gjkCd.getCollisionPairsBufferPtr();
const collisionPairsSize = gjkCd.getCollisionPairsBufferSize();

const rbIdOffsetMin = gjkCd.queryRbIdOffset(abId);
const rbIdOffsetMax = rbIdOffsetMin + gjkCd.queryRbLength(abId);
```


### 形状registration関係
```
const abPointer = self.CdModule.ab_alloc(packed.abLayer.length, abId);
const rbPointer = self.CdModule.rb_alloc(packed.rbLayer.length, abId);
const saPointer = self.CdModule.sa_alloc(packed.saLayer.length, abId);
const chPointer = self.CdModule.vertex_alloc(packed.vertices.length,...);
```

```
self.gjkCd = new CdModule.cd_constructor();
self.gjkCd.addLinkShape2(abId, abLayer.length, rbLayer.length,...);
```

```
self.CdModule.ab_free(abId);
self.CdModule.rb_free(abId);
self.CdModule.sa_free(abId);
self.CdModule.vertex_free(abId);
```



# 以下、旧worker.jsで使用
```
CdModule.delete(); // WASMモジュールを解放
CdModule.setJsLogLevel(data.logLevel);
gjkCd = new CdModule.CollisionDetection(jointModelVector,...);
gjkCd.heapF64 = CdModule.HEAPF64;
const shapeWasm = new CdModule.ConvexShapeVector();

gjkCd.addLinkShape(i, shapeWasm);
gjkCd.infoLinkShapes();
gjkCd.prepareStorage();
gjkCd.clearTestPairs();
gjkCd.addTestPair(pair[0],pair[1]);
gjkCd.clearTestPairs();
gjkCd.addTestPair(pair[0],pair[1]);
```



