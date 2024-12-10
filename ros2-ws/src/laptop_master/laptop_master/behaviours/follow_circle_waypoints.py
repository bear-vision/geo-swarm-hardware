from follow_waypoints import FollowWaypoints
import py_trees
from py_trees.common import Status

class FollowCircleWaypoints(FollowWaypoints):
    def __init__(self, behaviour_name, blackboard_waypoint_key):
        super().__init__(behaviour_name, blackboard_waypoint_key)
        self.blackboard.register_key("finished_circle_layer", access=py_trees.common.Access.READ)
        
    def setup(self, **kwargs):
        return super().setup(**kwargs)
    
    def initialise(self):
        return super().initialise()
    
    def update(self) -> Status:
        """Retrieve waypoints from blackboard and calls a ROS Action to nagivate to one waypoint at a time.

        Returns:
            Status.FAILURE we failed to reach a waypoint
            Status.SUCCESS if all waypoints were reached
            Status.RUNNING still traversing thru given waypoints
        """
        if self.waypoints is None:
            self.logger.info(f"Retrieving from blackboard: waypoints[{self.bb_waypoints_key}]")
            waypoints_dict = getattr(self.blackboard, 'waypoints', {})
            waypoints_dict_value = waypoints_dict.get(self.bb_waypoints_key)
            if not waypoints_dict_value:
                self.logger.error(f"waypoints[{self.bb_waypoints_key}] not found")
                return Status.FAILURE
            self.waypoints = waypoints_dict_value.poses
            
            return Status.RUNNING
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.logger.info("All waypoints reached")
            self.blackboard.finished_circle_layer = True
            return Status.SUCCESS
            
        # send one waypoint
        if self._send_goal_future is None:
            self.send_next_waypoint()
            return Status.RUNNING
        
        # still waiting for result
        if not self._get_result_future:
            self.logger.info(f"Still waiting for waypoint #{self.current_waypoint_index + 1} of {len(self.waypoints)}")
            return Status.RUNNING
        
        if self.action_result is not None:
            if self.action_result.success:
                self.logger.info(f"Reached waypoint #{self.current_waypoint_index + 1} of {len(self.waypoints)}")
                self.current_waypoint_index += 1
                self._send_goal_future = None
                self._get_result_future = None
                self.action_result = None
                return Status.RUNNING
            else:
                self.logger.error(f"Failed to reach waypoint {self.current_waypoint_index + 1}")
                return Status.FAILURE

        return Status.RUNNING
    
    def terminate(self, new_status):
        return super().terminate(new_status)