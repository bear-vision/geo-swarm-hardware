# #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose
from cv_bridge import CvBridge
import numpy as np
import cv2
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('tower_detector')

        # Initialize CVBridge for image conversions
        self.bridge = CvBridge()

        # Variables to store images
        self.cv_color_image = None
        self.cv_depth_image = None

        # Intrinsic camera parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Subscriptions
        self.color_image_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.color_image_callback,
            10
        )
        self.depth_image_sub = self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.depth_image_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            10
        )

        # Publishers
        # self.point_pub = self.create_publisher(Point, "tower_goal_point", 10)
        self.pose_sub = self.create_publisher(Pose, "tower_center_pose", 10)
        self.image_pub = self.create_publisher(Image, 'detected_tower', 10)

        self.get_logger().info("Object Detector Node Initialized")

    def camera_info_callback(self, msg):
        # Extract intrinsic parameters from CameraInfo
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info("Camera intrinsic parameters received.")

    def pixel_to_point(self, u, v, depth):
        # Convert pixel coordinates to real-world coordinates
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def color_image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (BGR8 format)
            self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info("Received a color image.")

            # If both color and depth images are available, process them
            if self.cv_depth_image is not None:
                self.process_images()
        except Exception as e:
            self.get_logger().error(f"Color Image Callback Error: {e}")

    def depth_image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (16UC1 format)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.get_logger().info("Received a depth image.")
        except Exception as e:
            self.get_logger().error(f"Depth Image Callback Error: {e}")

    def process_images(self):
        # Convert the color image to HSV color space
        hsv = cv2.cvtColor(self.cv_color_image, cv2.COLOR_BGR2HSV)

        # Define HSV range for the cup
        lower_hsv = np.array((10, 91, 0))  # Adjust these based on your object
        upper_hsv = np.array((73, 255, 255))

        # Threshold the image to get the cup mask
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Get coordinates of the mask
        y_coords, x_coords = np.nonzero(mask)

        if len(x_coords) == 0 or len(y_coords) == 0:
            self.get_logger().warn("No points detected. Check your color filter.")
            return

        # Calculate the center of the detected region
        center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))
        self.get_logger().info(f"Center coordinates: (x, y) = ({center_x}, {center_y})")

        # Fetch the depth value at the center
        depth = self.cv_depth_image[center_y, center_x]

        if self.fx and self.fy and self.cx and self.cy:
            camera_x, camera_y, camera_z = self.pixel_to_point(center_x, center_y, depth)

            # Convert from mm to meters
            camera_x /= 1000
            camera_y /= 1000
            camera_z /= 1000

            self.get_logger().info(f"Real-world coordinates: (X, Y, Z) = ({camera_x:.2f}m, {camera_y:.2f}m, {camera_z:.2f}m)")

            # Publish the point
            # self.point_pub.publish(Point(x=camera_x, y=camera_y, z=camera_z))

            # Publish the pose
            pose_msg = Pose()
            pose_msg.position.x = float(camera_x)
            pose_msg.position.y = float(camera_y)
            pose_msg.position.z = float(camera_z)
            pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = quaternion_from_euler(0, 0, 0)
            print(pose_msg)
            print(f'The quaternion representation is x: {pose_msg.orientation.x} y: {pose_msg.orientation.y} z: {pose_msg.orientation.z} w: {pose_msg.orientation.w}.')
            self.pose_sub.publish(pose_msg)

            # Overlay cup points on the color image for visualization
            cup_img = self.cv_color_image.copy()
            cup_img[y_coords, x_coords] = [0, 0, 255]  # Highlight points in red
            cv2.circle(cup_img, (center_x, center_y), 5, [0, 255, 0], -1)

            # Debug: Save the mask and processed image
            # cv2.imwrite("mask_debug.jpg", mask)
            # cv2.imwrite("cup_img_debug.jpg", cup_img)
            # self.get_logger().info("Debug images saved: mask_debug.jpg, cup_img_debug.jpg")

            # Publish the detected image
            try:
                ros_image = self.bridge.cv2_to_imgmsg(cup_img, "bgr8")
                self.image_pub.publish(ros_image)
                self.get_logger().info("Published detected cup image.")
            except Exception as e:
                self.get_logger().error(f"Failed to publish image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
