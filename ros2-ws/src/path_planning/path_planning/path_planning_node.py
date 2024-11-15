#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from scipy.interpolate import CubicSpline
import numpy as np
import math

from custom_srv_interfaces.srv import PathPlanner_Paint  # Change this to your actual service type
from custom_srv_interfaces.srv import PathPlanner_Up  # Change this to your actual service type
from custom_srv_interfaces.srv import PathPlanner_Spin  # Change this to your actual service type

class PathPlannerServiceNode(Node):
    def __init__(self):
        super().__init__('path_planner_service_node')

        # Define the service to plan a path based on start and target poses
        self.plan_path_service = self.create_service(
            PathPlanner_Paint, 'plan_path_paint', self.generate_waypoints_towards_paint)
        
        self.plan_path_service = self.create_service(
            PathPlanner_Up, 'plan_path_up', self.generate_waypoints_spin_around_tower)
        
        self.plan_path_service = self.create_service(
            PathPlanner_Spin, 'plan_path_spin', self.generate_waypoints_spin_around_tower)

    # def handle_plan_path_request(self, request, response):
    #     # Here we assume the service request provides start and target poses as arguments
    #     self.current_pose = request.start_pose
    #     self.target_pose = request.target_pose

    #     # request.scenario gives information whether we go up, spin, or towards paint -> to edit

    #     # Plan the path based on the received start and target poses
    #     waypoints = self.generate_waypoints_towards_paint(self)

    #     # Convert waypoints to PoseArray
    #     waypoint_array = PoseArray()
    #     waypoint_array.poses = waypoints

    #     # Return the waypoints in the response
    #     response.waypoints = waypoint_array  # Make sure your service type includes a PoseArray field

    #     self.get_logger().info('Waypoints generated and sent via service.')
    #     return response

    def generate_waypoints_towards_paint(self, request, response, num_waypoints=100):
        # response incude start and goal position
        # Generate timestamps for the waypoints
        t = np.linspace(0, 1, num_waypoints)

        # Start and goal positions for x, y, z, and yaw angle
        start_x = request.current_pose.position.x
        start_y = request.current_pose.position.y
        start_z = request.current_pose.position.z
        
        goal_x = request.target_pose.position.x
        goal_y = request.target_pose.position.y
        goal_z = request.target_pose.position.z

        start_yaw = self.quaternion_to_yaw(request.current_pose.orientation)
        goal_yaw = self.quaternion_to_yaw(request.target_pose.orientation)

        # Create smooth path using cubic splines for x, y, and z coordinates
        x_spline = CubicSpline([0, 1], [start_x, goal_x])
        y_spline = CubicSpline([0, 1], [start_y, goal_y])
        z_spline = CubicSpline([0, 1], [start_z, goal_z])
        yaw_spline = CubicSpline([0, 1], [start_yaw, goal_yaw])

        # Generate waypoints from the splines
        waypoints = []
        for ti in t:
            x = x_spline(ti)
            y = y_spline(ti)
            z = z_spline(ti)
            yaw = yaw_spline(ti)

            # Convert yaw to quaternion
            quaternion = self.yaw_to_quaternion(yaw)

            # Create a Pose with full 3D position and orientation
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation = quaternion
            waypoints.append(pose)

        response.waypoints
        return response
    
    def generate_waypoints_spin_around_tower(self, request, response, num_waypoints=100):
        # response incudes start position
        # Generate timestamps for the waypoints
        t = np.linspace(0, 1, num_waypoints)

        # Start and goal positions for x, y, z, and yaw angle
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start_z = self.current_pose.position.z
        
        goal_x = self.target_pose.position.x
        goal_y = self.target_pose.position.y
        goal_z = self.target_pose.position.z

        # Calculate yaw for smooth orientation transitions
        start_yaw = self.quaternion_to_yaw(self.current_pose.orientation)  # Replace with actual yaw if available
        goal_yaw = self.quaternion_to_yaw(self.target_pose.orientation)  # Replace with actual yaw if available

        # Create smooth path using cubic splines for x, y, and z coordinates
        x_spline = CubicSpline([0, 1], [start_x, goal_x])
        y_spline = CubicSpline([0, 1], [start_y, goal_y])
        z_spline = CubicSpline([0, 1], [start_z, goal_z])
        yaw_spline = CubicSpline([0, 1], [start_yaw, goal_yaw])

        # Generate waypoints from the splines
        waypoints = []
        for ti in t:
            x = x_spline(ti)
            y = y_spline(ti)
            z = z_spline(ti)
            yaw = yaw_spline(ti)

            # Convert yaw to quaternion
            quaternion = self.yaw_to_quaternion(yaw)

            # Create a Pose with full 3D position and orientation
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation = quaternion
            waypoints.append(pose)

        response.waypoints
        return response
    
    def generate_waypoints_going_up(self, request, response, num_waypoints=100):
        # response incudes start position
        # Generate timestamps for the waypoints
        t = np.linspace(0, 1, num_waypoints)

        # Start and goal positions for x, y, z, and yaw angle
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start_z = self.current_pose.position.z
        
        goal_x = self.target_pose.position.x
        goal_y = self.target_pose.position.y
        goal_z = self.target_pose.position.z

        # Calculate yaw for smooth orientation transitions
        start_yaw = self.quaternion_to_yaw(self.current_pose.orientation)  # Replace with actual yaw if available
        goal_yaw = self.quaternion_to_yaw(self.target_pose.orientation)  # Replace with actual yaw if available

        # Create smooth path using cubic splines for x, y, and z coordinates
        x_spline = CubicSpline([0, 1], [start_x, goal_x])
        y_spline = CubicSpline([0, 1], [start_y, goal_y])
        z_spline = CubicSpline([0, 1], [start_z, goal_z])
        yaw_spline = CubicSpline([0, 1], [start_yaw, goal_yaw])

        # Generate waypoints from the splines
        waypoints = []
        for ti in t:
            x = x_spline(ti)
            y = y_spline(ti)
            z = z_spline(ti)
            yaw = yaw_spline(ti)

            # Convert yaw to quaternion
            quaternion = self.yaw_to_quaternion(yaw)

            # Create a Pose with full 3D position and orientation
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation = quaternion
            waypoints.append(pose)

        response.waypoints
        return response

    def yaw_to_quaternion(self, yaw):
        # Convert yaw to quaternion (for ROS orientation)
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def quaternion_to_yaw(quaternion):
        #Converts a quaternion to a yaw angle (in radians).
        # Extract the quaternion components
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w

        # Calculate yaw
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return yaw

def main(args=None):
    rclpy.init(args=args)
    path_planner_service_node = PathPlannerServiceNode()
    rclpy.spin(path_planner_service_node)
    path_planner_service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()