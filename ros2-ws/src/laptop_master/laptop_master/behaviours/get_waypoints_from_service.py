"""NOT TESTED"""
import py_trees
from py_trees.common import Status
from geometry_msgs.msg import Pose, Quaternion
import math

class GetWaypointsFromService(py_trees.behaviour.Behaviour):
    def __init__(self, behaviour_name, service_type, service_name, blackboard_waypoint_key):
        super().__init__(behaviour_name)
        self.waypoints = None
        self.waypoint_client = None
        self.future = None
        self.service_name = service_name
        self.service_type = service_type
        self.blackboard_waypoint_key = f"waypoints/{blackboard_waypoint_key}"
        
        # read blackboard for current drone position and tower position
        self.blackboard = self.attach_blackboard_client() 
        self.blackboard.register_key("drone/position/x", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/position/y", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/position/z", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/orientation/yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/valid/xy_valid", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/valid/z_valid", access=py_trees.common.Access.READ)
        self.blackboard.register_key("tower_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(self.blackboard_waypoint_key, access=py_trees.common.Access.WRITE)
        
        
    def setup(self, **kwargs) -> None:
        """Sets up service.
        
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
        # self.tower_waypoint_client = self.node.create_client(PathPlannerSpin, 'plan_path_spin')
        self.waypoint_client = self.node.create_client(self.service_type, self.service_name)
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Waiting for the path planning service...')
            
            
    def initialise(self) -> None:
        """Creates service request. This function is called on the first tick each time this node enters a RUNNING state"""
        request = self.service_type.Request()
        
        # Retrieve current pose from blackboard (ENU TO NED)
        if (self.blackboard.drone.valid.xy_valid and self.blackboard.drone.valid.z_valid):
            request.current_pose.position.x = self.blackboard.drone.position.y
            request.current_pose.position.y = self.blackboard.drone.position.x
            request.current_pose.position.z = -self.blackboard.drone.position.z
            request.current_pose.orientation = yaw_to_quaternion(self.blackboard.drone.orientation.yaw)
        else:
            self.logger.error("Invalid drone x,y,z current pose. Is 'fmu/out/vehicle_local_positoin' topic available?")
                
        try:
            # Retrieve tower pose from blackboard (in NED)
            goal_pose = Pose()
            tower = self.blackboard.tower_position
            goal_pose.position.x = tower.position.x
            goal_pose.position.y = tower.position.y
            goal_pose.position.z = tower.position.z
            goal_pose.orientation.x = tower.orientation.x
            goal_pose.orientation.x = tower.orientation.y
            goal_pose.orientation.x = tower.orientation.z
            goal_pose.orientation.x = tower.orientation.w
            request.goal_pose = goal_pose
            
            # Request waypoints from service and save in blackboard for other behaviors to use
            self.future = self.waypoint_client.call_async(request)
            self.logger.info(f"Requested path planning service for {self.service_name}")
            
        except KeyError as e:
            self.logger.error("No tower pose found. Is perception topic available?")
        
       
    
    
    def update(self) -> Status:
        """Check if service request was completed and retrieves the waypoints

        Returns:
            Status.FAILURE if invalid x,y,z drone pose, or service call fails, or there is no service client yet 
            Status.SUCCESS if waypoints retrieved successfully
            Status.RUNNING if we are waiting for service response
        """
        self.logger.debug("{}.update()".format(self.qualified_name))
        
        if self.future is None:
            self.logger.error("No path planner service client set yet")
            return Status.FAILURE
        
        if not self.future.done():
            return Status.RUNNING
        
        try:
            response = self.future.result()
            self.waypoints = response.waypoints
            setattr(self.blackboard, self.blackboard_waypoint_key, self.waypoints) # this is self.blackboard.waypoints.to_tower = self.waypoints
            self.logger.info(f"Waypoints stored in blackboard key: {self.blackboard_waypoint_key}")
            return Status.SUCCESS
        except Exception as e:
            self.logger.error(f'Service call failed: {str(e)}')
            return Status.FAILURE
        
    # def get_waypoints(self):
    #     return self.waypoints

        
# TODO: maybe put these in a common library
def yaw_to_quaternion(yaw):
    # Convert yaw to quaternion (for ROS orientation)
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

