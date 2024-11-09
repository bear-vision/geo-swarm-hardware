import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DronePerceptionNode(Node):

    def __init__(self):
        super().__init__('drone_perception_node') #node name must be alphanumeric or _
        self.get_logger().info("Test print from DronePerceptionNode's __init__")

        self.color_img_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_image_callback,
            10
        )
        
        self.depth_img_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        #example publisher - currently written to detect the green cups from lab 8
        self.example_publisher = self.create_publisher(
            Image,
            '/detected_cup',
            10
        )

        self.bridge = CvBridge()

        self.fx = None
        self.fy = None 
        self.cx = None
        self.cy = None

        self.img_height = None
        self.img_width = None

        self.cv_color_img = None
        self.cv_depth_img = None

        print("Setup drone perception node")

    #msg of type RGBD
    def color_image_callback(self, msg):         

        try:
            self.cv_color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if self.cv_depth_img is not None:
                self.process_images()

        except Exception as e:
            print("Error in color image callback: ", e)


    def depth_image_callback(self, msg):
        try:
            self.cv_depth_img = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            print("Error in depth image callback: ", e)

    #msg of type CameraInfo
    def camera_info_callback(self, msg):
        #test
        print(f"Camera Info: images of size {msg.height} x {msg.width}")
        #extract relevant camera info for our use - this part is straight from lab 8
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.img_height = msg.height
        self.img_width = msg.width


    def process_images(self):
        '''
        Eventually, we want this to take in a message with RGBD info 
        (probably an RGBD frame) and return where the cardboard tower 
        is in relation to the camera.

        Currently written to detect the green cups from lab 8.
        '''
        #TODO - rewrite this to detect a tower and get the transform from it to the camera/drone.
        hsv = cv2.cvtColor(self.cv_color_img, cv2.COLOR_BGR2HSV)

        lower_hsv = np.array([77, 75, 36])
        upper_hsv = np.array([86, 255, 255])

        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        y_coords, x_coords = np.nonzero(mask)
        center_y, center_x = int(np.mean(y_coords)), int(np.mean(x_coords))

        depth = self.cv_depth_img[center_y, center_x]
        print(f"Cup detected at depth {depth}")

        #publish to our example pub

        cup_img = self.cv_color_img.copy()
        cup_img[y_coords, x_coords] = [0, 0, 255]
        cv2.circle(cup_img, (center_x, center_y), 5, [0, 255, 0], -1)
        ros_img = self.bridge.cv2_to_imgmsg(cup_img, 'bgr8')
        self.example_publisher.publish(ros_img)
        
        return

def main(args=None):
    rclpy.init(args=args)
    perception_node = DronePerceptionNode()
    rclpy.spin(perception_node)

    perception_node.destroy_node()
    rclpy.shutdown()

    #realsense node launch command - ros2 launch realsense2_camera rs_launch.py

if __name__ == '__main__':
    main()
