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
