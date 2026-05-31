#!/usr/bin/env python3
"""BlueROV2 torpedo mission entry point.

Builds `create_torpedo_root()` from `bluerov_tasks.torpedo.torpedo` (mirrored
from mission_planner_2's AUV4 RoboSub-24 torpedo tree) and tick-tocks it
with a BumbleTree.

`world_to_torp_yaw` is the yaw from the world frame to the torpedo panel —
configured for the `robosub_2025_pool` world below. Adjust via launch arg
once we expose a parameter; for now it's a constant matching the panel
orientation in `bb_worlds/worlds/robosub_2025_pool.world`.

Prereqs: launch `bluerov_torpedo.launch.py` so the locomotion action server,
convert_to_controls_pose service, cluster_tf action+service servers, static
TFs, choice_server, vision pipeline, and the actuators stub are all up.
"""

import math
import traceback

import py_trees
import py_trees.console as console
import rclpy
from bluerov_tasks.node_registry import BlueROVTreeNode
from bluerov_tasks.torpedo.torpedo import create_torpedo_root
from mission_planner_2.common.core.bumble_tree import BumbleTree
from mission_planner_2.common.core.hooks import stop_on_success_or_failure

TICK_PERIOD_MS = 100
WORLD_TO_TORP_YAW_RAD = math.radians(0.0)   # tune to torpedo panel SDF pose
ZERO_YAW_KEY = "/global/zero_yaw"


def main(args=None) -> None:
    rclpy.init(args=args)
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    root = create_torpedo_root(
        world_to_torp_yaw=WORLD_TO_TORP_YAW_RAD,
        zero_yaw_key=ZERO_YAW_KEY,
    )
    tree = BumbleTree(root=root)
    node = BlueROVTreeNode("bluerov_torpedo_mission_tree")

    try:
        tree.setup(node=node, timeout=30.0)
    except Exception:
        console.logerror(
            console.red + "Failed to set up the torpedo behaviour tree:\n"
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
