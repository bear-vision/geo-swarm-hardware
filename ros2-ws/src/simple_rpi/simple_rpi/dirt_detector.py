#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32, Bool, Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('dirt_detector')

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
        self.pose_array_pub = self.create_publisher(PoseArray, "dirt_patch_poses", 10)
        self.radii_array_pub = self.create_publisher(Float32MultiArray, "dirt_patch_radii", 10)
        self.image_pub = self.create_publisher(Image, 'detected_dirt', 10)
        self.is_dirt_pub = self.create_publisher(Bool, 'is_dirt_detected', 10)

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

        # Define HSV range for detecting dirt patches
        lower_hsv = np.array((0, 179, 124))
        upper_hsv = np.array((30, 228, 255))

        # Threshold the image to get the mask
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.erode(mask, kernel)
        mask = cv2.dilate(mask, kernel)

        is_dirty = Bool()
        is_dirty.data = False

        pose_array = PoseArray()
        radii_array = Float32MultiArray()

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for contour in contours:
            # Calculate area and filter small patches
            area = cv2.contourArea(contour)
            if area <= 500:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue

            # Calculate circularity
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            if 0.3 < circularity < 1.5:
                # Compute the center of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])

                    # Fetch the depth value at the center
                    depth = self.cv_depth_image[center_y, center_x]

                    if self.fx and self.fy and self.cx and self.cy:
                        X, Y, Z = self.pixel_to_point(center_x, center_y, depth)

                        # Create Pose for this patch
                        pose_msg = Pose()
                        pose_msg.position.x = float(X / 1000)
                        pose_msg.position.y = float(Y / 1000)
                        pose_msg.position.z = float(Z / 1000)
                        pose_msg.orientation.x = float(0)
                        pose_msg.orientation.y = float(0)
                        pose_msg.orientation.z = float(0)
                        pose_msg.orientation.w = float(1)
                        print(pose_msg)
                        pose_array.poses.append(pose_msg)
                        print(pose_array)

                        # Calculate radius
                        distances = np.sqrt((np.array([p[0][0] for p in contour]) - center_x) ** 2 +
                                            (np.array([p[0][1] for p in contour]) - center_y) ** 2)
                        radius = np.max(distances)
                        radii_array.data.append(float(radius))

                        # Draw center and contour on the image
                        cv2.circle(self.cv_color_image, (center_x, center_y), 5, (255, 0, 0), -1)
                        cv2.drawContours(self.cv_color_image, [contour], -1, (0, 255, 0), 2)
                        is_dirty.data = True

        # Publish PoseArray and radii array
        self.pose_array_pub.publish(pose_array)
        self.radii_array_pub.publish(radii_array)

        # Publish processed image
        try:
            ros_image = self.bridge.cv2_to_imgmsg(self.cv_color_image, "bgr8")
            self.image_pub.publish(ros_image)
            self.is_dirt_pub.publish(is_dirty)
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
