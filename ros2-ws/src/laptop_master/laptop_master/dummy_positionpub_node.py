"""
For testing
"""

import rclpy
from rclpy.node import Node
from time import sleep
from px4_msgs.msg import VehicleLocalPosition
from custom_interfaces.msg import PerceptionStuff


class PositionPubNode(Node):
    def __init__(self):
        super().__init__('position_pub_node')
        self.get_logger().info("Initialize dummy pub node.")
        
        self.pos_pub = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', 10)
        self.perception_pub = self.create_publisher(PerceptionStuff, 'realsense/out/perception_stuff', 10)
        self.timer = self.create_timer(0.2, self.timer_callback) # 5 Hz
        self.i = 0.0
        
    def timer_callback(self):
        pos_msg = VehicleLocalPosition()
        pos_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000.0)
        pos_msg.x = float(self.i)
        pos_msg.y = float(self.i)
        pos_msg.z = float(self.i)
        pos_msg.heading = 3.14
        pos_msg.xy_valid = True    # Add validity flags
        pos_msg.z_valid = True
        self.pos_pub.publish(pos_msg)
        self.get_logger().debug(f"Published position: (x={self.i}, y={self.i}, z={self.i})")
        self.i += 1.0
        
        perception_msg = PerceptionStuff()
        perception_msg.detected_tower = True
        perception_msg.tower_position.position.x = 10.0
        perception_msg.tower_position.position.y = 10.0
        perception_msg.tower_position.position.z = 0.0
        perception_msg.tower_position.orientation.x = 0.0
        perception_msg.tower_position.orientation.y = 0.0
        perception_msg.tower_position.orientation.z = 0.0
        perception_msg.tower_position.orientation.w = 0.0
        self.perception_pub.publish(perception_msg)
        self.get_logger().info(f"Published tower: (x=10.0, y=10.0, z=0.0)")
    

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
