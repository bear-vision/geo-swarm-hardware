#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from scipy.interpolate import CubicSpline
import numpy as np
import math

from custom_interfaces.srv import PathPlannerPaint, PathPlannerUp, PathPlannerSpin, PathPlannerTower

class PathPlannerServiceNode(Node):
    def __init__(self):
        super().__init__('path_planner_service_node')

        # Define the service to plan a path based on start and target poses
        self.plan_path_service = self.create_service(
            PathPlannerPaint, 'plan_path_paint', self.generate_waypoints_towards_paint)
        
        self.plan_path_service = self.create_service(
            PathPlannerTower, 'plan_path_tower', self.generate_waypoints_towards_tower)
        
        self.plan_path_service = self.create_service(
            PathPlannerUp, 'plan_path_up', self.generate_waypoints_going_up)
        
        self.plan_path_service = self.create_service(
            PathPlannerSpin, 'plan_path_spin', self.generate_waypoints_spin_around_tower)

    def generate_waypoints_towards_tower(self, request, response):
        num_waypoints = request.num_waypoints
        if num_waypoints <= 0:
            num_waypoints = 10
        
        

        # Extract current and target positions
        curr_x = request.current_pose.position.x
        curr_y = request.current_pose.position.y
        curr_z = request.current_pose.position.z

        tower_x = request.tower_pose.position.x
        tower_y = request.tower_pose.position.y
        tower_z = request.current_pose.position.z

        # Circle properties (radius and number of points)
        #TODO - handle bad radius. setting to have a minimum r for now
        radius = max(request.radius, 1.8)
        circle_num_points = 1000

        # Generate circle waypoints around the tower
        circle_waypoints = [
            (
                tower_x + radius * math.cos(2 * math.pi * i / circle_num_points),
                tower_y + radius * math.sin(2 * math.pi * i / circle_num_points)
            )
            for i in range(circle_num_points)
        ]

        # Find the closest circle waypoint to the current position
        closest_waypoint = min(
            circle_waypoints,
            key=lambda wp: math.sqrt((wp[0] - curr_x) ** 2 + (wp[1] - curr_y) ** 2)
        )
        closest_x, closest_y = closest_waypoint

        # Generate timestamps for the waypoints
        t = np.linspace(0, 1, num_waypoints)

        # Use cubic splines to interpolate between current and closest circle position
        start_velocity = 0
        end_velocity = 0
        x_spline = CubicSpline([0, 1], [curr_x, closest_x], bc_type=((1, start_velocity), (1, end_velocity)))
        y_spline = CubicSpline([0, 1], [curr_y, closest_y], bc_type=((1, start_velocity), (1, end_velocity)))
        z_spline = CubicSpline([0, 1], [curr_z, tower_z], bc_type=((1, start_velocity), (1, end_velocity)))

        # Generate waypoints
        waypoints = []
        for ti in t:
            x = x_spline(ti)
            y = y_spline(ti)
            z = z_spline(ti)

            # Calculate yaw to face the tower
            yaw = math.atan2(tower_y - y, tower_x - x)
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

        # Log progress
        self.get_logger().info('Waypoints generated and sent via service.')
        self.get_logger().info('Lets go to the tower...')
        return response

    def generate_waypoints_towards_paint(self, request, response):
        num_waypoints = request.num_waypoints
        if num_waypoints <= 1:
            #TODO - handle the case where we have an invalid number of waypoints. Setting to 10 for now
            num_waypoints = 5

        # response incude start and goal position
        # Generate timestamps for the waypoints
        t = np.linspace(0, 1, num_waypoints)[1:]

        # need a radius to stay away from the paint so we don't hit the tower... need to tune
        radius_from_paint = 1.0

        # Current and target positions for x, y, z, and yaw angle
        curr_x = request.current_pose.position.x
        curr_y = request.current_pose.position.y
        curr_z = request.current_pose.position.z
        
        paint_x = request.target_pose.position.x
        paint_y = request.target_pose.position.y
        paint_z = request.target_pose.position.z

        curr_yaw = quaternion_to_yaw(request.current_pose.orientation)
        #the target orientation has dummy values. TODO. use the orientation part of the target pose properly
        target_yaw = math.atan2(paint_y - curr_y, paint_x - curr_x)
        
        target_x = paint_x - radius_from_paint * np.cos(target_yaw)
        target_y = paint_y - radius_from_paint * np.sin(target_yaw)
        target_z = paint_z
        # target_yaw = quaternion_to_yaw(request.target_pose.orientation)

        #actual target posn calculation - to the radius distance away from paint pos

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
    
    def generate_waypoints_going_up(self, request, response):

        height_diff = request.height_diff

        num_waypoints = request.num_waypoints + 1
        if num_waypoints <= 1:
            #TODO - handle the case where we have an invalid number of waypoints. Setting to 3 for now
            num_waypoints = 3

        # response incude start and goal position
        

        # Generate timestamps for the waypoints 
        t = np.linspace(0, 1, num_waypoints)[1:]

        # Current and target positions for x, y, z, and yaw angle
        curr_x = request.current_pose.position.x
        curr_y = request.current_pose.position.y
        curr_z = request.current_pose.position.z
        
        # Target position only differs in z since we want to just move to the next level of the tower
        target_z = request.current_pose.position.z + height_diff

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
    
    def generate_waypoints_spin_around_tower(self, request, response, transition_fraction=0.2):

        num_waypoints = request.num_waypoints
        radius = request.radius
        if num_waypoints <= 0:
            #TODO - handle the case where we have an invalid number of waypoints. Setting to 25 for now
            num_waypoints = 25
        
        #TODO - handle bad radius. setting to have a minimum r for now
        radius = max(radius, 1.8)
        

        # Extract current and tower positions
        self.get_logger().info(f"Request current pose: {request.current_pose}, request tower pose: {request.tower_pose}\n")
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

        self.get_logger().info(f"Current radius: {current_radius}. Desired radius: {radius}\n")

        if np.isclose(current_radius, radius, atol=0.2):
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
        if np.isclose(current_radius, radius, atol=0.2):
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