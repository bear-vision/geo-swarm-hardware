from geometry_msgs.msg import Pose, Quaternion
import numpy as np
import tf2_geometry_msgs

def px4_to_ros_transform(pose: Pose):
    """Transform a pose from PX4 frame to ROS frame."""
    # TODO implement
    return pose

def ros_to_px4_transform(pose: Pose):
    """Transform a pose from ROS frame to PX4 frame."""
    # TODO implement
    return pose

def euler_from_quaternion(quaternion: Quaternion):
    # Convert the quaternion to Euler angles (roll, pitch, yaw)
    return tf2_geometry_msgs.transformations.euler_from_quaternion(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    )

    