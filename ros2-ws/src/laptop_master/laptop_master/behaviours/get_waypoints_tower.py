import py_trees
from py_trees.common import Status
from geometry_msgs.msg import Pose, Quaternion
import math

class GetWaypointsTower(py_trees.behaviour.Behaviour):
    def __init__(self, behaviour_name, service_type, service_name, blackboard_waypoint_key):
        super().__init__(behaviour_name)
        self.waypoints = None
        self.waypoint_client = None
        self.future = None
        self.service_name = service_name
        self.service_type = service_type
        self.blackboard_waypoint_key = blackboard_waypoint_key
        
        # read blackboard for current drone position and tower position
        self.blackboard = self.attach_blackboard_client() 
        self.blackboard.register_key("drone/position/x", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/position/y", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/position/z", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/orientation/yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/valid/xy_valid", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/valid/z_valid", access=py_trees.common.Access.READ)
        self.blackboard.register_key("tower/position", access=py_trees.common.Access.READ)
        self.blackboard.register_key("tower/orientation", access=py_trees.common.Access.READ)
        self.blackboard.register_key("waypoints", access=py_trees.common.Access.WRITE)
        
        
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
            self.logger.info(f'Waiting for {self.service_name} service...')
            
            
    def initialise(self) -> None:
        """Creates service request. This function is called on the first tick each time this node enters a RUNNING state"""
        request = self.service_type.Request()
        
        try:
            # Retrieve current pose from blackboard (PX4 to ROS2 frame)
            if (self.blackboard.drone.valid.xy_valid and self.blackboard.drone.valid.z_valid):
                request.current_pose.position.x = self.blackboard.drone.position.y
                request.current_pose.position.y = self.blackboard.drone.position.x
                request.current_pose.position.z = max(-self.blackboard.drone.position.z, 0.0)
                or_x, or_y, or_z, or_w = yaw_to_quaternion(self.blackboard.drone.orientation.yaw)
                request.current_pose.orientation.x = float(or_x)
                request.current_pose.orientation.y = float(or_y)
                request.current_pose.orientation.z = float(or_z)
                request.current_pose.orientation.w = float(or_w)

                self.logger.info(f"Current drone pose: {(request.current_pose.position.x, request.current_pose.position.y, request.current_pose.position.z)}")
            else:
                self.logger.error("Invalid drone x,y,z current pose.")
        except KeyError as e:
            self.logger.error(f"No local position found. Is vehicle local position topic available? {str(e)}")
        
        try:
            # Retrieve tower pose from blackboard (in NED - PX4 frame)
            goal_pose = Pose()
            goal_pose.position = self.blackboard.tower.position
            goal_pose.orientation = self.blackboard.tower.orientation
            request.tower_pose = goal_pose        
        except KeyError as e:
            self.logger.error(f"No tower pose found. Is perception topic available? {str(e)}")
        except Exception as err:
            self.logger.error(f"More errors with perception blackboard... {str(err)}")
        
        # Request waypoints from service and save in blackboard for other behaviors to use
        self.future = self.waypoint_client.call_async(request)
        self.logger.info(f"Requested {self.service_name} service")       
    
    
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
            waypoints_dict = {self.blackboard_waypoint_key: self.waypoints}
            setattr(self.blackboard, 'waypoints', waypoints_dict) # obcject.attribute = value
            self.logger.info(f"Waypoints stored in blackboard dict: waypoints[{self.blackboard_waypoint_key}]")
            return Status.SUCCESS
        except Exception as e:
            self.logger.error(f'Service call failed: {str(e)}')
            return Status.FAILURE
        
    # def get_waypoints(self):
    #     return self.waypoints

        
# TODO: maybe put these in a common library
def yaw_to_quaternion(yaw):
    # Convert yaw to quaternion (for ROS orientation)
    # q = Quaternion()
    # q.z = float(math.sin(yaw / 2.0))
    # q.w = float(math.cos(yaw / 2.0))
    return 0, 0, float(math.sin(yaw / 2.0)), float(math.cos(yaw / 2.0))

