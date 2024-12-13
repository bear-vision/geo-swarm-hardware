import py_trees
from py_trees.common import Status
from geometry_msgs.msg import Pose
import math

class GetWaypointsOnePaintBlob(py_trees.behaviour.Behaviour):
    def __init__(self, behaviour_name, service_type, service_name, blackboard_waypoint_key):
        super().__init__(behaviour_name)
        self.waypoints = None
        self.waypoint_client = None
        self.future = None
        self.service_name = service_name
        self.service_type = service_type
        self.blackboard_waypoint_key = blackboard_waypoint_key

        
        # read blackboard for current drone position and paint blob position
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("drone/position/x", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/position/y", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/position/z", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/orientation/yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/valid/xy_valid", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/valid/z_valid", access=py_trees.common.Access.READ)
        self.blackboard.register_key("paint/positions", access=py_trees.common.Access.READ)        
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
        self.waypoint_client = self.node.create_client(self.service_type, self.service_name)
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.logger.info(f'Waiting for {self.service_name} service...')
    
    
    def initialise(self) -> None:
        """Creates the service request"""
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
            # Retrieve paint position
            paint_positions = self.blackboard.paint.positions
            request.target_pose = paint_positions[0] # TODO - we are assuming one waypoint rn
            self.blackboard.paint_target = paint_positions[0] # first point
        except KeyError as e:
            self.logger.error(f"No paint pose found. {str(e)}")
        except Exception as err:
            self.logger.error(f"More errors with perception blackboard... {str(err)}")
            
        # Request waypoints from service
        self.future = self.waypoint_client.call_async(request)
        self.logger.info(f"Requested {self.service_name} service")       
            
    
    def update(self) -> Status:
        """Check if service request was completed and retrieves the waypoints

        Returns:
            Status.FAILURE if service call fails, or there is no service client yet 
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
        
# TODO: maybe put these in a common library
def yaw_to_quaternion(yaw):
    # Convert yaw to quaternion (for ROS orientation)
    # q = Quaternion()
    # q.z = float(math.sin(yaw / 2.0))
    # q.w = float(math.cos(yaw / 2.0))
    return 0, 0, float(math.sin(yaw / 2.0)), float(math.cos(yaw / 2.0))

