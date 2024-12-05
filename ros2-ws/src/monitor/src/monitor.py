#!/usr/bin/python

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition, VehicleStatus, VehicleAttitude

class PX4Subscriber(Node):

    def __init__(self):
        super().__init__('px4_subscriber')

        # Subscribe to PX4 topics
        self.sub_control_mode = self.create_subscription(
            VehicleControlMode, 
            '/fmu/out/vehicle_control_mode', 
            self.control_mode_callback, 
            10
        )
        self.sub_local_position = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.local_position_callback, 
            10
        )
        self.sub_local_position = self.create_subscription(
            VehicleAttitude, 
            '/fmu/out/vehicle_attitude', 
            self.vehicle_attitude_callback, 
            10
        )
        self.sub_vehicle_status = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 
            10
        )

    def control_mode_callback(self, msg):
        self.get_logger().info(f"Control Mode: {msg}")

    def local_position_callback(self, msg):
        self.get_logger().info(f"Local Position - X: {msg.x}, Y: {msg.y}, Z: {msg.z}")
    def vehicle_attitude_callback(self, msg):
        self.get_logger().info(f"attitude quaternion: {msg.q}")    
    def vehicle_status_callback(self, msg):
        self.get_logger().info(f"preflight check: {msg.pre_flight_checks_pass}")



def main(args=None):
    rclpy.init(args=args)
    node = PX4Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
