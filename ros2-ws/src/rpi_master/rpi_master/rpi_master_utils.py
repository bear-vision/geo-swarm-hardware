from geometry_msgs.msg import Pose, Quaternion, Transform, TransformStamped
import numpy as np
import tf2_geometry_msgs
import transforms3d

# both transforms to the relative frames can be represented with rotation about axis and angle { [ 0.7071068, 0.7071068, 0 ], 3.1415927 }
ROS2_PX4_UNIVERSAL_WORLD_FRAME_TRANSFORM = TransformStamped(transform = Transform(rotation = Quaternion(x = np.sqrt(2) / 2, y = np.sqrt(2) / 2, z = 0.0, w = 0.0)))

def px4_to_ros_world_frame_transform(pose: Pose):
    """Transform a pose (in vehicle local frame) from PX4 coordinates to ROS coordinates."""
    # TODO implement
    return tf2_geometry_msgs.do_transform_pose(pose, ROS2_PX4_UNIVERSAL_WORLD_FRAME_TRANSFORM)

def ros_to_px4_world_frame_transform(pose: Pose):
    """Transform a pose (in vehicle local frame) from ROS coordinates to PX4 coordinates."""
    # TODO implement
    return tf2_geometry_msgs.do_transform_pose(pose, ROS2_PX4_UNIVERSAL_WORLD_FRAME_TRANSFORM)

def euler_from_quaternion(quaternion: Quaternion):
    # Convert the quaternion to Euler angles (roll, pitch, yaw)
    return transforms3d.euler.quat2euler([quaternion.w, quaternion.x, quaternion.y, quaternion.z]) 

    