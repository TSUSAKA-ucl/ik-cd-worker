## ik-workerがmessage channel(self)から受け付けるコマンドの順序関係

1. `init`に先立ってでも何時でも実行できるもの
   * `shutdown`	省略
   * `set_worker_loglevel`	省略
   * `set_slrm_loglevel`	省略
   * `set_cd_loglevel`	省略
   * `init` 別に解説

2. after `init` completed
   * `set_initial_joints`
   * `set_exact_solution`
   * `set_joint_weights`
   * `set_joint_desirable_vlimit`
   * `clear_joint_desirable`
   * `set_joint_desirable`
   * `set_joint_velocity_limit`
   * `set_ignore_collisions`
   * `set_ignore_joint_limits`
   * `set_joint_limit_keep_moving`

3. 動作開始ok後(`set_initial_joints`)
   * `destination`
   * `set_joint_targets`
   * `slow_rewind`

4. IK実行後(`destination`)
   * `set_end_effector_point`
   * `set_end_effector_position`
   * `set_end_effector_orientation`
   * `set_end_effector_pose`
