"""
For testing
"""

import rclpy
from rclpy.node import Node
from time import sleep
from geometry_msgs.msg import Point, Quaternion, Pose
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from custom_interfaces.msg import PerceptionStuff
from rpi_master import rpi_master_utils
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class DummyPubNode(Node):
    def __init__(self):
        super().__init__('position_pub_node')
        self.get_logger().debug("Initialize dummy pub node.")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #paint locations in ROS2 frame as defined in the PX4 autopilot tower model file
        self.paint_locations = [Point(x = 0.475, y = 0.0, z = 1.0), Point(x = -0.475, y = 0.0, z = 2.0), Point(x = 0.0, y = 0.475, z = 6.0), Point(x = 0.0, y = -0.475, z = 9.0)]
        
        # stores local position in PX4 frame
        self.drone_posn = VehicleLocalPosition()
        self.drone_orientation = VehicleAttitude()

        self.drone_pose = Pose()

        # continuously update drone local position (PX4 frame)
        self.drone_orientation_subscriber = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        self.drone_posn_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        
        self.perception_pub = self.create_publisher(PerceptionStuff, 'realsense/out/perception_stuff', 10)
        
        self.timer = self.create_timer(0.2, self.timer_callback) # 5 Hz
        
        self.i = 0.0
        
    def timer_callback(self):
        
        perception_msg = PerceptionStuff()
        perception_msg.detected_tower = True
        perception_msg.tower_position.position.x = 10.0
        perception_msg.tower_position.position.y = 10.0
        perception_msg.tower_position.position.z = 10.0
        perception_msg.tower_position.orientation.x = 0.0
        perception_msg.tower_position.orientation.y = 0.0
        perception_msg.tower_position.orientation.z = 0.0
        perception_msg.tower_position.orientation.w = 1.0

        #TODO - If self.drone_pose is close enough to a paint blob in self.paint_locations, then set relevant fields in Perception msg
        # (i think a good starting radius to test is 1.8 meters - we can change it later)


        self.perception_pub.publish(perception_msg)
        self.get_logger().debug(f"Published tower: (x=10.0, y=10.0, z=10.0)")
    
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Update vehicle local position."""
        self.drone_posn = vehicle_local_position
        self.update_vehicle_local_pose()

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Update vehicle attitude (orientation)."""
        self.drone_orientation = vehicle_attitude

    def update_vehicle_local_pose(self):
        """
            Takes the vehicle local position and vehicle attitude (both in PX4 frame) and converts to ROS2 frame.
        """
        #we assume that the current local position and vehicle attitude are time-synced.

        if self.drone_posn and self.drone_orientation:
            px4_drone_pose = Pose()
            px4_drone_pose.position.x = self.drone_posn.x
            px4_drone_pose.position.y = self.drone_posn.y
            px4_drone_pose.position.z = self.drone_posn.z
            px4_drone_pose.orientation.w = float(self.drone_orientation.q[0])
            px4_drone_pose.orientation.x = float(self.drone_orientation.q[1])
            px4_drone_pose.orientation.y = float(self.drone_orientation.q[2])
            px4_drone_pose.orientation.z = float(self.drone_orientation.q[3])

            ros2_drone_pose = rpi_master_utils.px4_to_ros_world_frame_transform(px4_drone_pose)
            return ros2_drone_pose

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
