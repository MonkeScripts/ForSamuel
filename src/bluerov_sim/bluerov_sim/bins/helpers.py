import numpy as np
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion

from mission_planner_2.common.util.pose_utils import create_stamped_pose


def find_acute_angle(pose: PoseStamped) -> PoseStamped:
    r, p, y = euler_from_quaternion(
        [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
    )

    y = y - np.pi if y > np.pi / 2 else y

    return create_stamped_pose(
        frame_id=pose.header.frame_id,
        position_x=pose.pose.position.x,
        position_y=pose.pose.position.y,
        position_z=pose.pose.position.z,
        roll=r,
        pitch=p,
        yaw=y,
        use_radians=True,
    )


def within_threshold(
    xyz_pose: PoseStamped,
    rpy_pose: PoseStamped,
    distance_threshold: float,
    yaw_threshold: float,
) -> bool:
    _, _, y = euler_from_quaternion(
        [
            rpy_pose.pose.orientation.x,
            rpy_pose.pose.orientation.y,
            rpy_pose.pose.orientation.z,
            rpy_pose.pose.orientation.w,
        ]
    )

    y = np.degrees(y) % 360

    if y > yaw_threshold:
        return False

    distance = np.sqrt(
        (xyz_pose.pose.position.x**2)
        + (xyz_pose.pose.position.y**2)
        + (xyz_pose.pose.position.z**2)
    )

    if distance > distance_threshold:
        return False

    return True
