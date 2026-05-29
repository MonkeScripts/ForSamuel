import py_trees

from mission_planner_2.common.util.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.common.util.pose_utils import create_stamped_pose
from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto

# Generate namespace automatically from file path
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


def create_move_to_torpedo_task_root(world_coords: dict):
    """
    Create the root of the torpedo tree.
    """
    root = py_trees.composites.Sequence(
        name="Move to torpedo task",
        memory=True,
    )

    torpedo_init_pose = create_stamped_pose(
        "base_link",
        position_x=world_coords["x"],
        position_y=world_coords["y"],
        position_z=world_coords["z"],
        roll=world_coords["roll"],
        pitch=world_coords["pitch"],
        yaw=world_coords["yaw"],
    )

    move_to_torp = goto.FromConstant(
        "Move to torpedo",
        torpedo_init_pose,
        anchor_frame_name="base_link",
    )

    move_to_torp_rel = goto.FromConstant(
        "Move to torpedo",
        create_stamped_pose(
            "base_link",
            yaw=-90.0,
        ),
        anchor_frame_name="base_link",
    )

    # TODO: figure out the move to board subtree
    root.add_children(
        children=[
            move_to_torp
            # move_to_torp_rel,
        ]
    )

    return root
