#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from scipy.interpolate import CubicSpline
import numpy as np
import math

from custom_interfaces.srv import PathPlannerPaint
from custom_interfaces.srv import PathPlannerUp
from custom_interfaces.srv import PathPlannerSpin

class PathPlannerServiceNode(Node):
    def __init__(self):
        super().__init__('path_planner_service_node')

        # Define the service to plan a path based on start and target poses
        self.plan_path_service = self.create_service(
            PathPlannerPaint, 'plan_path_paint', self.generate_waypoints_towards_paint)
        
        self.plan_path_service = self.create_service(
            PathPlannerUp, 'plan_path_up', self.generate_waypoints_going_up)
        
        self.plan_path_service = self.create_service(
            PathPlannerSpin, 'plan_path_spin', self.generate_waypoints_spin_around_tower)

    def generate_waypoints_towards_paint(self, request, response):
        num_waypoints = request.num_waypoints
        if num_waypoints <= 0:
            #TODO - handle the case where we have an invalid number of waypoints. Setting to 10 for now
            num_waypoints = 10

        # response incude start and goal position
        # Generate timestamps for the waypoints
        t = np.linspace(0, 1, num_waypoints)

        # Current and target positions for x, y, z, and yaw angle
        curr_x = request.current_pose.position.x
        curr_y = request.current_pose.position.y
        curr_z = request.current_pose.position.z
        
        target_x = request.target_pose.position.x
        target_y = request.target_pose.position.y
        target_z = request.target_pose.position.z

        curr_yaw = quaternion_to_yaw(request.current_pose.orientation)
        target_yaw = quaternion_to_yaw(request.target_pose.orientation)

        # Set velocity  values (same as the linear trajectory code)
        start_velocity = 0  # Start velocity
        end_velocity = 0   # End velocity

        # Create cubic splines for each coordinate with velocity  boundary conditions
        x_spline = CubicSpline([0, 1], [curr_x, target_x], bc_type=((1, start_velocity), (1, end_velocity)))
        y_spline = CubicSpline([0, 1], [curr_y, target_y], bc_type=((1, start_velocity), (1, end_velocity)))
        z_spline = CubicSpline([0, 1], [curr_z, target_z], bc_type=((1, start_velocity), (1, end_velocity)))
        yaw_spline = CubicSpline([0, 1], [curr_yaw, target_yaw], bc_type=((1, start_velocity), (1, end_velocity)))

        # Generate waypoints using the splines
        waypoints = []
        for ti in t:
            x = x_spline(ti)
            y = y_spline(ti)
            z = z_spline(ti)
            yaw = yaw_spline(ti)

            # Convert yaw to quaternion
            quaternion = yaw_to_quaternion(yaw)

            # Create a Pose message for each waypoint
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation = quaternion
            waypoints.append(pose)

        # Populate the response with the waypoints
        waypoint_array = PoseArray()
        waypoint_array.poses = waypoints
        response.waypoints = waypoint_array

        # Send message to console
        self.get_logger().info('Waypoints generated and sent via service.')
        self.get_logger().info('Lets clean the paint...')
        return response
    
    def generate_waypoints_going_up(self, request, response, heightDiff=1):

        num_waypoints = request.num_waypoints
        if num_waypoints <= 0:
            #TODO - handle the case where we have an invalid number of waypoints. Setting to 5 for now
            num_waypoints = 5

        # response incude start and goal position
        

        # Generate timestamps for the waypoints
        t = np.linspace(0, 1, num_waypoints)

        # Current and target positions for x, y, z, and yaw angle
        curr_x = request.current_pose.position.x
        curr_y = request.current_pose.position.y
        curr_z = request.current_pose.position.z
        
        # Target position only differs in z since we want to just move to the next level of the tower
        target_z = request.current_pose.position.z + heightDiff

        # extract
        curr_orientation = request.current_pose.orientation

        # Set velocity values (same as the linear trajectory code)
        start_velocity = 0  # Start velocity
        goal_velocity = 0   # End velocity

        # Create cubic splines for each coordinate with velocity boundary conditions
        z_spline = CubicSpline([0, 1], [curr_z, target_z], bc_type=((1, start_velocity), (1, goal_velocity)))
        
        # Generate waypoints using the splines
        waypoints = []
        for ti in t:
            z = z_spline(ti)

            # Create a Pose message for each waypoint
            pose = Pose()
            pose.position.x = curr_x
            pose.position.y = curr_y
            pose.position.z = float(z)
            pose.orientation = curr_orientation
            waypoints.append(pose)

        # Populate the response with the waypoints
        waypoint_array = PoseArray()
        waypoint_array.poses = waypoints
        response.waypoints = waypoint_array

        # Send message to console
        self.get_logger().info('Waypoints generated and sent via service.')
        self.get_logger().info('We are going up...')
        return response
    
    def generate_waypoints_spin_around_tower(self, request, response, radius=1.2, transition_fraction=0.2):

        num_waypoints = request.num_waypoints
        if num_waypoints <= 0:
            #TODO - handle the case where we have an invalid number of waypoints. Setting to 25 for now
            num_waypoints = 25

        # Extract current and tower positions
        curr_x = request.current_pose.position.x
        curr_y = request.current_pose.position.y
        curr_z = request.current_pose.position.z

        tower_x = request.tower_pose.position.x
        tower_y = request.tower_pose.position.y

        # Calculate the current radius and start angle
        current_radius = np.sqrt((curr_x - tower_x) ** 2 + (curr_y - tower_y) ** 2)
        start_angle = np.arctan2(curr_y - tower_y, curr_x - tower_x)

        # Initialize waypoints list
        waypoints = []

        if np.isclose(current_radius, radius, atol=1e-3):
            # === First Scenario: Already on the Circle ===
            self.get_logger().info('Scenario 1: Starting on the circle.')
            # Generate time steps for the full trajectory
            t = np.linspace(0, 1, num_waypoints)
            angles = start_angle + 2 * np.pi * t

            for angle in angles:
                # Compute x, y, z positions based on the current angle and radius
                x = tower_x + radius * np.cos(angle)
                y = tower_y + radius * np.sin(angle)
                z = curr_z

                # Compute the yaw angle (always facing the tower)
                yaw = np.arctan2(tower_y - y, tower_x - x)
                quaternion = yaw_to_quaternion(yaw)

                # Create a Pose message for each waypoint
                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                pose.position.z = float(z)
                pose.orientation = quaternion

                waypoints.append(pose)
        else:
            # === Second Scenario: Moving Towards the Circle ===
            self.get_logger().info('Scenario 2: Moving towards the circle.')
            # Define the number of waypoints for the transition phase
            num_transition_points = int(transition_fraction * num_waypoints)
            t = np.linspace(0, 1, num_waypoints)

            # Gradually transition the radius from current to desired over the transition phase
            radius_values = np.piecewise(
                t,
                [t <= transition_fraction, t > transition_fraction],
                [
                    lambda t: current_radius + (radius - current_radius) * (t / transition_fraction),
                    lambda t: radius,
                ],
            )

            # Generate angular values for a full 360° rotation
            angles = start_angle + 2 * np.pi * t

            for i in range(num_waypoints):
                # Compute x, y, z positions based on the current angle and radius
                x = tower_x + radius_values[i] * np.cos(angles[i])
                y = tower_y + radius_values[i] * np.sin(angles[i])
                z = curr_z

                # Compute the yaw angle (always facing the tower)
                yaw = np.arctan2(tower_y - y, tower_x - x)
                quaternion = yaw_to_quaternion(yaw)

                # Create a Pose message for each waypoint
                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                pose.position.z = float(z)
                pose.orientation = quaternion

                waypoints.append(pose)

        # Populate the response with the waypoints
        waypoint_array = PoseArray()
        waypoint_array.poses = waypoints
        response.waypoints = waypoint_array

        # Log the scenario executed
        if np.isclose(current_radius, radius, atol=1e-3):
            self.get_logger().info('Circular trajectory complete for Scenario 1.')
        else:
            self.get_logger().info('Dynamic circular trajectory complete for Scenario 2.')

        return response

def yaw_to_quaternion(yaw):
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