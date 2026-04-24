* 2026-04-23:  
  * [x] unitree G1の指にアームとの間のignore-collisionをつける
  * [x] send-base-coordがattach-to-anotherの後tickが再起動するようにregisterResetTargetする
  * [x] attach-to-anotherにsetTimeout(()=> {el.play();}, 0);を入れるが絶対的な効果は無い(だいたいは良い)
  * [x] attach-to-anotherがel.axesでなくel.realAxesに基づいて付けるリンクを決められるようにする(位置ずれ防止)
  * [?] stop dependencies対応
    * [?] attach-to-anotherは、cd-workerがstop dependency listを作れるように、子(自分)のworkerのabIdをターゲット(親)robotのik-workerに{command:'stop_dependency', stopAbId: abId}で伝える
	* [?] ik-workerはcd-workerにそのままRPCで流す
	* [?] cd-workerは、abStopDependencies配列をメンテしてdependenciesツリーを構築し、collideしたrbがdependenciesに入っている場合もcollision listに含めて返す(リンク番号は負か範囲外になる)
    * [?] ik-workerは自分にlength>0のcollision listが帰ってくればrewindする
	* [?] reflect-collisionは自分の範囲内のリンク番号にだけ色を付ける
* 2026-04-22:  
  * [ ] unitreeの指の`attach_to_another`する場所がフランジ(fixed jointの先)になっていないorできない
  * [ ] `attach_to_another`のbase link座標の変更が、根本側を動かす(destination)までik-workerに伝わらずcd-workerにも伝わっていないかも。
  * [ ] unitreeの指のreflect_collisionが変。現象自体要調査
  * [ ] 指どうしが接触した場合、アームは動いてしまう? 要調査。ストップのbubble upを検討するか。
* 2026-04-10: バグ修正とコード管理
  * [x] `set_initial_joints`だけでは、`cdPort`に`rb_poses`が送られない問題の解決
  * [x] ik計算が無い場合の`cdPort`への`rb_poses`の送信方法の検討と実装  
	* ik-workerはdestinationが来なければ十分負荷が低いか確認
    * 案: ハンドにもik-workerを付ける。`fingerCloser`はworkerが付いていれば`set_joint_targets`
	  を送る。`set_joint_targets`でのcdチェック確認
	* floorなど固定物を直接cd-workerに送りつけるルート  
	  メインでjsonをfetchしてab構造を作りメインから送る。メインは複数abに対応可能なプロトコル必要
  * [1] 親子関係の干渉情報のリターン方法を決めて実装
  
