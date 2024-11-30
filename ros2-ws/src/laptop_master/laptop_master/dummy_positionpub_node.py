"""
For testing
"""

import rclpy
from rclpy.node import Node
from time import sleep
from px4_msgs.msg import VehicleLocalPosition


class PositionPubNode(Node):
    def __init__(self):
        super().__init__('position_pub_node')
        self.get_logger().info("Initialize position pub node.")
        
        self.pub = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', 10)
        self.timer = self.create_timer(0.2, self.timer_callback) # 5 Hz
        self.i = 0.0
        
    def timer_callback(self):
        msg = VehicleLocalPosition()
        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000.0)
        msg.x = float(self.i)
        msg.y = float(self.i)
        msg.z = float(self.i)
        msg.heading = 3.14
        msg.xy_valid = True    # Add validity flags
        msg.z_valid = True
        
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing: (x={self.i}, y={self.i}, z={self.i})")
        self.i += 1.0
    

def main(args = None):
    # Initialize node
    rclpy.init(args = args)
    node = PositionPubNode()   
    
    try: 
        rclpy.spin(node) # keep node running
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
