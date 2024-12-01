import py_trees
from py_trees.common import Status
from custom_srv_interfaces.srv import PathPlannerSpin
from geometry_msgs.msg import Pose, Quaternion
import math

class GetWaypointsToTower(py_trees.behaviour.Behaviour):
    def __init__(self, name="Get waypoints to tower"):
        super().__init__(name)
        self.waypoints = None
        self.tower_waypoint_client = None
        self.blackboard = self.attach_blackboard_client() # create blackboard client
        self.blackboard.register_key("position/x", access=py_trees.common.Access.READ)
        self.blackboard.register_key("position/y", access=py_trees.common.Access.READ)
        self.blackboard.register_key("position/z", access=py_trees.common.Access.READ)
        self.blackboard.register_key("orientation/yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key("valid/xy_valid", access=py_trees.common.Access.READ)
        self.blackboard.register_key("valid/z_valid", access=py_trees.common.Access.READ)

        
    def setup(self, **kwargs) -> None:
        """Sets up PathPlannerSpin service.
        
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
        
        # Set up service client
        self.tower_waypoint_client = self.node.create_client(PathPlannerSpin, 'plan_path_spin')
        while not self.tower_waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the plan_path_spin service...')
            
            
    def initialise(self) -> None:
        """Create a service request."""
        request = PathPlannerSpin.Request()
        # Retrieve current pose from blackboard (ENU TO NED)
        if (self.blackboard.valid.xy_valid and self.blackboard.valid.z_valid):
            request.current_pose.position.x = self.blackboard.position.y
            request.current_pose.position.y = self.blackboard.position.x
            request.current_pose.position.z = -self.blackboard.position.z
            request.current_pose.orientation = yaw_to_quaternion(self.blackboard.orientation.yaw)
        else:
            self.logger.error("GetWayPointsToTower: invalid drone x,y,z current pose")
            return Status.FAILURE
        # TODO: retrieve tower pose from blackboard, using dummy for now
        goal_pose = Pose()
        goal_pose.position.x = 2.0  
        goal_pose.position.y = 3.0
        goal_pose.position.z = 0.0
        goal_pose.orientation.w = 0.0
        
        # Request waypoints from service and save in blackboard for other behaviors to use
        self.future = self.tower_waypoint_client.call_async(request)
        self.logger.info(f'called service with current position:')
    
    
    def update(self) -> Status:
        """Check if service request was completed and retrieves the waypoints

        Returns:
            Status: FAILURE if invalid x,y,z drone pose, or service call fails. SUCCESS if waypoints retrieved.
        """
        self.logger.debug("{}.update()".format(self.qualified_name))
        
        if not self.future.done():
            print("not done yet")
            return Status.RUNNING
        
        try:
            response = self.future.result()
            self.waypoints = response.waypoints
            self.logger.info(f'GetWayPointsToTower: retrieved waypoints:')
            print(self.waypoints)
            return Status.SUCCESS
        except Exception as e:
            self.logger.error(f'GetWayPointsToTower: Service call failed: {str(e)}')
            return Status.FAILURE
        
    def get_waypoints(self):
        return self.waypoints

        
# TODO: maybe put these in a common library
def yaw_to_quaternion(yaw):
    # Convert yaw to quaternion (for ROS orientation)
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

