"""Forked subset of mission_planner_2.vehicles.shared.trees.

Only forks files that contain hardcoded AUV4 frame/service names which
prevented direct reuse on BlueROV2. Everything else (BumbleTree, hooks,
pose_utils, cluster_goto, blackboard, tf_checker, goto helpers) is still
imported from upstream `mission_planner_2`.

Diff vs. upstream: frame name (`auv4/base_link_ned` → `base_link`) and
ClusterTfSrv service path (`/auv4/cluster_tfs_srv` → `/bluerov/cluster_tfs_srv`)
are now keyword-arg-parameterised with BlueROV defaults. `anchor_frame_name`
is also passed through to every `goto.*FromConstant`-style call.
"""
