import py_trees
from py_trees.common import Status
from geometry_msgs.msg import Pose
from custom_interfaces.action import DroneNavigateToWaypoint
from rclpy.action import ActionClient
import math

class FollowWaypoints(py_trees.behaviour.Behaviour):
    def __init__(self, behaviour_name, blackboard_waypoint_key):
        super().__init__(behaviour_name)
        self.waypoints = None
        self.waypoint_index = 0
        self.action_client = None
        self.action_result = None
        self._get_result_future = None
        self.bb_waypoints_key = blackboard_waypoint_key
        self.blackboard = self.attach_blackboard_client() 
        self.blackboard.register_key(self.bb_waypoints_key, access=py_trees.common.Access.READ)

        
    def setup(self, **kwargs) -> None:
        """Sets up the DroneNavigateToWaypoint ROS action client.
        
        Args:
            **kwargs (dict): look for the 'node' object being passed down from the tree
            
        Raises:
            KeyError: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        # Get node from the tree
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.action_client = ActionClient(self.node, DroneNavigateToWaypoint, '/drone_navigate_to_waypoint')
        
    def initialise(self):
        self.current_waypoint_index = 0 
        self._send_goal_future = None
        
    def update(self) -> Status:
        """Retrieve waypoints from blackboard and calls a ROS Action to nagivate to one waypoint at a time.

        Returns:
            Status.FAILURE we failed to reach a waypoint
            Status.SUCCESS if all waypoints were reached
            Status.RUNNING still traversing thru given waypoints
        """
        if self.waypoints is None:
            self.logger.info("Retrieved waypoints from blackboard")
            self.waypoints = self.blackboard.waypoints.to_tower.poses
            return Status.RUNNING
        
        if self.waypoint_index >= len(self.waypoints):
            self.logger.info("All waypoints reached")
            return Status.SUCCESS
            
        # send one waypoint
        if self._send_goal_future is None:
            self.send_next_waypoint()
            return Status.RUNNING
        
        # still waiting for result
        if not self._get_result_future:
            self.logger.debug(f"Still waiting for waypoint #{self.current_waypoint_index}")
            return Status.RUNNING
        
        if self.action_result is not None:
            if self.action_result.success:
                self.logger.info(f"Reached waypoint #{self.current_waypoint_index}")
                self.current_waypoint_index += 1
                self._send_goal_future = None
                self._get_result_future = None
                self.action_result = None
                return Status.RUNNING
            else:
                self.logger.error(f"Failed to reach waypoint {self.current_waypoint_index}")
                return Status.FAILURE
        
        return Status.RUNNING

    def send_next_waypoint(self):
        goal_msg = DroneNavigateToWaypoint.Goal()
        goal_msg.waypoint = self.waypoints[self.current_waypoint_index]
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.logger.info(f"Sending goal for waypoint #{self.current_waypoint_index}: "
                        f"({goal_msg.waypoint.position.x}, {goal_msg.waypoint.position.y}, {goal_msg.waypoint.position.z}), "
                        f"yaw ({quaternion_to_yaw(goal_msg.waypoint.orientation)})")

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.error(f"Goal for waypoint #{self.current_waypoint_index} was rejected")
            self._get_result_future = None
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
        
    def get_result_callback(self, future):
        self.action_result = future.result().result


    def terminate(self, new_status) -> None:
        """This is called whenever your node switches to a non running state (SUCCESS, FAILURE, or INVALID)

        Args:
            new_status: stateus we terminate  the node with 
        """
        self.logger.info(f"Terminating with new status: {new_status}")
        if self.goal_handle and self.goal_handle.is_active():
            self.logger.info("Cancelling current goal")
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
            
            
# TODO: maybe put this in a common library
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