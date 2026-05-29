#!/usr/bin/env python3
"""BlueROV2 bin mission entry point.

Builds `create_bin_root()` from `bluerov_sim.bins.bins` (mirrored from
mission_planner_2's AUV4 RoboSub-24 bin tree) and tick-tocks it with a
BumbleTree.

Prereqs (each in its own tmux pane — see bluerov_bin_mission.yaml):
  • bluerov_sim.launch.py          (Gazebo + ArduSub + MAVROS)
  • bluerov_controls.launch.py     (locomotion + convert_to_controls_pose + TFs + actuators; shared)
  • bluerov_cluster.launch.py      (cluster_tf action + service servers under /bluerov; shared)
  • bluerov_bin_vision.launch.py   (YOLO + image_matching + pose estimators)
Then bluerov_bin_bt.launch.py runs this node.
"""

import traceback

import py_trees
import py_trees.console as console
import rclpy
from bluerov_sim.bins.bins import create_bin_root
from bluerov_sim.node_registry import BlueROVTreeNode
from mission_planner_2.common.core.bumble_tree import BumbleTree
from mission_planner_2.common.core.hooks import stop_on_success_or_failure

TICK_PERIOD_MS = 100  # 10 Hz


def main(args=None) -> None:
    rclpy.init(args=args)
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    root = create_bin_root()
    tree = BumbleTree(root=root)
    node = BlueROVTreeNode("bluerov_bin_mission_tree")

    try:
        tree.setup(node=node, timeout=30.0)
    except Exception:
        console.logerror(
            console.red + "Failed to set up the bin behaviour tree:\n"
            + traceback.format_exc() + console.reset
        )
        tree.shutdown()
        rclpy.try_shutdown()
        return

    tree.add_post_tick_handler(stop_on_success_or_failure)
    tree.tick_tock(period_ms=TICK_PERIOD_MS)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        console.loginfo(console.yellow + "interrupted" + console.reset)
    except SystemExit:
        console.loginfo(console.yellow + "mission finished — exiting" + console.reset)
    except Exception:
        console.logfatal(console.red + traceback.format_exc() + console.reset)
    finally:
        console.loginfo(console.reset + "cleaning up")
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
