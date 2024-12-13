"""
For testing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray
from custom_interfaces.msg import PerceptionStuff
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np

class DummyPubNode(Node):
    def __init__(self):
        super().__init__('dummy_pub_node')
        self.get_logger().debug("Initialize dummy pub node.")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #paint locations in ROS2 frame as defined in the PX4 autopilot tower model file
        self.paint_locations = [Point(x = -9.525, y = 10.0, z = 1.0), Point(x = -9.525, y = 10.0, z = 2.0), Point(x = 10.0, y = 10.475, z = 6.0), Point(x = 10.0, y = -9.525, z = 9.0)]
        
        self.perception_pub = self.create_publisher(PerceptionStuff, 'realsense/out/perception_stuff', 10)
        self.timer = self.create_timer(0.2, self.timer_callback) # 5 Hz


    def timer_callback(self):

        perception_msg = PerceptionStuff()
        perception_msg.detected_tower = True
        perception_msg.tower_position = Pose(position = Point(x = 10.0, y = 10.0, z = 10.0), orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0))
        
        perception_msg.detected_dirty_patch = True
        perception_msg.num_dirty_patches = 1
        dirty_patch_positions = PoseArray()
        dirty_patch_positions.poses = [Pose(position = Point(x = -9.525, y = 10.0, z = 1.0), orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0))]
        perception_msg.dirty_patches = dirty_patch_positions
        perception_msg.dirty_patches_radii = [0.5]

        self.perception_pub.publish(perception_msg)
    

def main(args = None):
    # Initialize node
    rclpy.init(args = args)
    node = DummyPubNode()   
    
    try: 
        rclpy.spin(node) # keep node running
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
