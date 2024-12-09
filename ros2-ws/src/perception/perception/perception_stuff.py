#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import Pose, PoseArray
# from std_msgs.msg import Float32, Bool, Float32MultiArray
# from perception_node_msgs.msg import PerceptionMsg
# from perception_node_msgs.msg import PercArray


# class MinimalSubscriber(Node):
#     def __init__(self):
#         super().__init__('perception_stuff_sub')

#         # Instance variables to store the subscribed data
#         self.dirt_center = Pose()
#         self.tower_center = Pose()
#         self.radi = 0.0

#         # Subscriptions
#         self.subscription1 = self.create_subscription(
#             Pose,
#             'dirt_center_pose',
#             self.listener_callback1,
#             10
#         )
#         self.subscription2 = self.create_subscription(
#             Pose,
#             'tower_center_pose',
#             self.listener_callback2,
#             10
#         )
#         self.subscription3 = self.create_subscription(
#             Float32,
#             'detected_radi_pub',
#             self.listener_callback3,
#             10
#         )

#         # Publisher
#         self.point_pub = self.create_publisher(PerceptionMsg, 'perception_stuff_pub', 10)

#         # Timer to periodically publish the PerceptionMsg
#         self.timer = self.create_timer(1.0, self.publish_perception_message)

#         self.get_logger().info("MinimalSubscriber initialized and ready.")

#     def listener_callback1(self, msg1):
#         self.dirt_center = msg1
#         # self.get_logger().info(f"Updated dirt_center: {self.dirt_center}")

#     def listener_callback2(self, msg2):
#         self.tower_center = msg2
#         # self.get_logger().info(f"Updated tower_center: {self.tower_center}")

#     def listener_callback3(self, msg3):
#         self.radi = msg3.data
#         # self.get_logger().info(f"Updated radi: {self.radi}")

#     def publish_perception_message(self):
#         # Create the PerceptionMsg with the latest data
#         perception_msg = PerceptionMsg()
#         perception_msg.dirt_position = self.dirt_center
#         perception_msg.tower_position = self.tower_center
#         perception_msg.dirty_patches_radii = float(self.radi)

#         # Publish the message
#         self.point_pub.publish(perception_msg)
#         self.get_logger().info(f"Published perception message: {perception_msg}")

# def main(args=None):
#     rclpy.init(args=args)

#     # minimal_subscriber = MinimalSubscriber()
#     minimal_subscriber = MinimalSubscriberArray()

#     try:
#         rclpy.spin(minimal_subscriber)
#     except KeyboardInterrupt:
#         minimal_subscriber.get_logger().info('Node stopped by user.')
#     finally:
#         # Clean up
#         minimal_subscriber.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray, Bool
from custom_interfaces.msg import PerceptionStuff
import time


class MinimalSubscriberArray(Node):
    def __init__(self):
        super().__init__('perception_stuff_sub')

        # Instance variables to store the subscribed data
        self.dirt_centers = PoseArray()
        self.tower_center = Pose()
        self.radii = Float32MultiArray()
        self.is_dirt = Bool()

        # Subscriptions
        self.subscription1 = self.create_subscription(
            PoseArray,
            'dirt_patch_poses',
            self.listener_callback1,
            10
        )
        self.subscription2 = self.create_subscription(
            Pose,
            'tower_center_pose',
            self.listener_callback2,
            10
        )
        self.subscription3 = self.create_subscription(
            Float32MultiArray,
            'dirt_patch_radii',
            self.listener_callback3,
            10
        )
        self.subscription4 = self.create_subscription(
            Bool,
            'is_dirt_detected',
            self.listener_callback4,
            10
        )

        # Publisher
        self.point_pub = self.create_publisher(PerceptionStuff, 'perception_stuff_pub', 10)

        # Timer to periodically publish the PerceptionMsg
        self.timer = self.create_timer(1.0, self.publish_perception_message)

        self.get_logger().info("MinimalSubscriberArray initialized and ready.")

    def listener_callback1(self, msg1):
        self.dirt_centers = msg1
        self.get_logger().info(f"Updated dirt centers: {len(self.dirt_centers.poses)} patches detected.")

    def listener_callback2(self, msg2):
        self.tower_center = msg2
        self.get_logger().info(f"Updated tower center: {self.tower_center}")

    def listener_callback3(self, msg3):
        self.radii = msg3
        self.get_logger().info(f"Updated radii: {self.radii.data}")
    
    def listener_callback4(self, msg4):
        self.is_dirt = msg4
        self.get_logger().info(f"Updated is_dirt: {self.is_dirt.data}")

    def publish_perception_message(self):
        # Create the PerceptionMsg with the latest data
        perception_msg = PerceptionStuff()
        perception_msg.num_dirty_patches = len(self.dirt_centers.poses)
        perception_msg.dirty_patches = self.dirt_centers
        perception_msg.tower_position = self.tower_center
        perception_msg.dirty_patches_radii = self.radii.data
        perception_msg.detected_dirty_patch = self.is_dirt.data

        # Publish the message
        self.point_pub.publish(perception_msg)
        time.sleep(5)
        self.get_logger().info("Published perception message.")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriberArray()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        minimal_subscriber.get_logger().info('Node stopped by user.')
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
