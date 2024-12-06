import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from custom_interfaces.action import DroneNavigateToWaypoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude, VehicleOdometry
from enum import Enum
import numpy as np
from rpi_master import rpi_master_utils

# for use when implementing multithreading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

# Lots of code in here is re-used from https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py     

# TODO:
# (IMPORTANT) implement MultiThreadedExecutor and use threading.lock to handle issues with resource sharing and concurrency
# Consider adding another action just for landing safely


class RPiTestNode(Node):

    def __init__(self):
        super().__init__('rpi_test_node')
        self.get_logger().info(f"Initialize RPi Test Node.")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Set up subscribers for Pixhawk
        self.vehicle_pose_subscriber = self.create_subscription(PoseStamped, '/vicon/citris_drone/citris_drone/pose', self.vehicle_pose_callback, 10)
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        

        # Set up publishers for Pixhawk
        self.vehicle_pose_publisher = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile)
        
        self.drone_pose = Pose()
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        

        self.create_timer(1 / 100, self.publish_drone_odometry)

    def destroy(self):
        super().destroy_node()
        
    def get_current_pose(self):
        """Get current pose of the drone as a geometry_msgs/msg/Pose object. Returns the current pose in PX4 frame."""
        # TODO use threading to handle concurrency correctly
        pose_obj = Pose()
        pose_obj.position.x = self.vehicle_local_position.x
        pose_obj.position.y = self.vehicle_local_position.y
        pose_obj.position.z = self.vehicle_local_position.z

        pose_obj.orientation.w = self.vehicle_attitude.q[0]
        pose_obj.orientation.x = self.vehicle_attitude.q[1]
        pose_obj.orientation.y = self.vehicle_attitude.q[2]
        pose_obj.orientation.z = self.vehicle_attitude.q[3]

        return pose_obj
    
    def vehicle_pose_callback(self, pose_stamped):
        self.drone_pose = pose_stamped

    def publish_drone_odometry(self):
        msg = VehicleOdometry()
        pos, ori = self.drone_pose.pose.position, self.drone_pose.pose.orientation

        msg.position = [pos.x, pos.y, pos.z]
        #quaternion is w, x, y, z for px4
        msg.q = [ori.w, ori.x, ori.y, ori.z]

        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_pose_publisher.publish(msg)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Update vehicle local position."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Update vehicle attitude (orientation)."""
        self.vehicle_attitude = vehicle_attitude
        # self.log_vehicle_pose_px4()

    def log_vehicle_pose_px4(self):
        posn = f"\n - x: {self.vehicle_local_position.x}\n - x: {self.vehicle_local_position.y}\n - x: {self.vehicle_local_position.z}"
        ori = f"\n - w: {self.vehicle_attitude.q[0]}\n - x: {self.vehicle_attitude.q[1]}\n - y: {self.vehicle_attitude.q[2]}\n - z: {self.vehicle_attitude.q[3]}"
        self.get_logger().info(f"Vehicle local position (PX4): {posn} \n Vehicle attitude (PX4): {ori}")


def main(args = None):
    # Initialize node
    rclpy.init(args = args)
    node = RPiTestNode()   
    
    try: 
        rclpy.spin(node) # keep node running
    except KeyboardInterrupt:
        node.get_logger().info("RPi Master Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
