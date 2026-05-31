import py_trees

from mission_planner_2.common.util.pose_utils import create_stamped_pose
from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto


def create_move_to_bin_task_root(world_coords: dict) -> py_trees.behaviour.Behaviour:
    """
    Create the root of the bin tree.
    """
    root = py_trees.composites.Sequence(
        name="Move to Bin Task Root",
        memory=True,
    )

    bin_init_pose = create_stamped_pose(
        "base_link",
        position_x=world_coords["x"],
        position_y=world_coords["y"],
        position_z=world_coords["z"],
        roll=world_coords["roll"],
        pitch=world_coords["pitch"],
        yaw=world_coords["yaw"],
    )

    move_to_bin = goto.FromConstant(
        "move to bin", bin_init_pose, anchor_frame_name="base_link"
    )

    root.add_children(
        [
            move_to_bin,
        ]
    )

    return root
