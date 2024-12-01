#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from custom_interfaces.srv import PathPlannerPaint, PathPlannerUp, PathPlannerSpin
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class PathPlannerClient(Node):
    def __init__(self):
        super().__init__('path_planner_client')
        # self.client = self.create_client(PathPlannerPaint, 'plan_path_paint')
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for the service...')
        
        # self.send_request()

        # self.client = self.create_client(PathPlannerUp, 'plan_path_up')
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for the service...')
        
        # self.send_request()

        self.client = self.create_client(PathPlannerSpin, 'plan_path_spin')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the service...')
        
        self.send_request()

    def send_request(self):
        # request = PathPlannerPaint.Request()
        # request = PathPlannerUp.Request()
        request = PathPlannerSpin.Request()
        
        # Initialize start and goal poses
        start_pose = Pose()
        start_pose.position.x = 0.0
        start_pose.position.y = 0.0
        start_pose.position.z = 1.0
        start_pose.orientation.w = 1.0  # Set orientation as needed
        
        goal_pose = Pose()
        goal_pose.position.x = 0.5
        goal_pose.position.y = 0.0
        goal_pose.position.z = 1.0
        goal_pose.orientation.w = 1.0  # Set orientation as needed

        request.current_pose = start_pose
        # request.target_pose = goal_pose
        request.tower_pose = goal_pose
        
        # Send request
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            waypoints = response.waypoints.poses
            
            # Extract x, y, z values from the waypoints
            x_vals = [pose.position.x for pose in waypoints]
            y_vals = [pose.position.y for pose in waypoints]
            z_vals = [pose.position.z for pose in waypoints]
            print(x_vals)

            # Create a 3D plot
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(x_vals, y_vals, z_vals, marker='o', label='Waypoints')
            
            # Add the start point
            ax.scatter(x_vals[0], y_vals[0], z_vals[0], color='green', s=100, label='Start Point')
        
            # Add the end point
            ax.scatter(x_vals[-1], y_vals[-1], z_vals[-1], color='red', s=100, label='End Point')

            # Set labels
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            # Set title
            ax.set_title('Waypoints Path')

            # Show plot
            plt.show()

            # Plot the results
            # plt.figure(figsize=(12, 6))

            # # Position plot
            # t = np.linspace(0, 1, 100)
            # plt.subplot(2, 2, 1)
            # plt.plot(t, x_vals, label="X Position")
            # plt.plot(t, y_vals, label="Y Position")
            # plt.plot(t, z_vals, label="Z Position")
            # plt.title("Position Along the Path")
            # plt.legend()

            # # Velocity plot (first derivative of position)
            # plt.subplot(2, 2, 2)
            # plt.plot(t, cs_x.derivative(1)(t), label="X Velocity")
            # plt.plot(t, cs_y.derivative(1)(t), label="Y Velocity")
            # plt.plot(t, cs_z.derivative(1)(t), label="Z Velocity")
            # plt.title("Velocity Along the Path")
            # plt.legend()

            # # Acceleration plot (second derivative of position)
            # plt.subplot(2, 2, 3)
            # plt.plot(t, cs_x.derivative(2)(t), label="X Acceleration")
            # plt.plot(t, cs_y.derivative(2)(t), label="Y Acceleration")
            # plt.plot(t, cs_z.derivative(2)(t), label="Z Acceleration")
            # plt.title("Acceleration Along the Path")
            # plt.legend()

            # plt.tight_layout()
            plt.show()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    path_planner_client = PathPlannerClient()
    rclpy.spin(path_planner_client)
    path_planner_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
