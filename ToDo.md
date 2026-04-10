* 2026-04-10: バグ修正とコード管理
  * [ ] `u2ur5e`の初期動作の不具合原因の調査と修正
  * [ ] `set_initial_joints`だけでは、`cdPort`に`rb_poses`が送られない問題の解決
  * [ ] ik計算が無い場合の`cdPort`への`rb_poses`の送信方法の検討と実装  
	* ik-workerはdestinationが来なければ十分負荷が低いか確認
    * 案: ハンドにもik-workerを付ける。`fingerCloser`はworkerが付いていれば`set_joint_targets`
	  を送る。`set_joint_targets`でのcdチェック確認
	* floorなど固定物を直接cd-workerに送りつけるルート  
	  メインでjsonをfetchしてab構造を作りメインから送る。メインは複数abに対応可能なプロトコル必要
  * [ ] 親子関係の干渉情報のリターン方法を決めて実装
  
