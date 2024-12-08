"""
For testing
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Pose
from custom_interfaces.action import DroneNavigateToWaypoint

class GoToWaypointActionServer(Node):

    def __init__(self):
        super().__init__('go_to_waypoint_action_server')
        self._action_server = ActionServer(
            self,
            DroneNavigateToWaypoint,
            '/drone_navigate_to_waypoint',
            self.execute_callback)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Print the goal pose
        goal_pose = goal_handle.request.waypoint
        self.get_logger().info(f'Goal Pose Received:')
        self.get_logger().info(f'Position: x={goal_pose.position.x}, y={goal_pose.position.y}, z={goal_pose.position.z}')
        self.get_logger().info(f'Orientation: x={goal_pose.orientation.x}, y={goal_pose.orientation.y}, z={goal_pose.orientation.z}, w={goal_pose.orientation.w}')

        # dummy -- always succeed
        result = DroneNavigateToWaypoint.Result()
        result.success = True
        result.message = "Waypoint reached successfully"
        goal_handle.succeed()
        
        return result

def main(args=None):
    rclpy.init(args=args)

    go_to_waypoint_server = GoToWaypointActionServer()

    try:
        rclpy.spin(go_to_waypoint_server)
    except KeyboardInterrupt:
        pass

    go_to_waypoint_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
