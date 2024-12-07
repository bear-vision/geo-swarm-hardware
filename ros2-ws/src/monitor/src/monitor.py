#!/usr/bin/python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition, VehicleStatus, VehicleAttitude

class PX4Subscriber(Node):

    def __init__(self):
        super().__init__('px4_subscriber')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to PX4 topics
        self.sub_control_mode = self.create_subscription(
            VehicleControlMode, 
            '/fmu/out/vehicle_control_mode', 
            self.control_mode_callback, 
            qos_profile
        )
        self.sub_local_position = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.local_position_callback, 
            qos_profile
        )
        self.sub_local_position = self.create_subscription(
            VehicleAttitude, 
            '/fmu/out/vehicle_attitude', 
            self.vehicle_attitude_callback, 
            qos_profile
        )
        self.sub_vehicle_status = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 
            qos_profile
        )

    def control_mode_callback(self, msg):
        self.get_logger().info(f"Control Mode: {msg}\n")

    def local_position_callback(self, msg):
        self.get_logger().info(f"Local Position \n - X: {msg.x} \n - Y: {msg.y} \n - Z: {msg.z}\n")
    def vehicle_attitude_callback(self, msg):
        self.get_logger().info(f"attitude quaternion: \n - {msg.q}\n")    
    def vehicle_status_callback(self, msg):
        self.get_logger().info(f"preflight check: {msg.pre_flight_checks_pass}\n")



def main(args=None):
    rclpy.init(args=args)
    node = PX4Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
