import py_trees
from py_trees.common import Status
from std_srvs.srv import Trigger

class LandDrone(py_trees.behaviour.Behaviour):
    def __init__(self, behaviour_name):
        super().__init__(behaviour_name)
        self.land_client = None
        self.service_type = Trigger
        self.service_name = '/rpi_master/land_drone'
        self.future = None


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
        self.land_client = self.node.create_client(self.service_type, self.service_name)
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.logger.info(f'Waiting for {self.service_name} service...')
            
            
    def initialise(self) -> None:
        """Creates service request. This function is called on the first tick each time this node enters a RUNNING state"""
        request = self.service_type.Request()
        
        # Tell drone to land
        self.future = self.land_client.call_async(request)
        self.logger.info(f"Requested {self.service_name} service")       
    
    
    def update(self) -> Status:
        """Check if service request was completed and update status

        Returns:
            Status.FAILURE if land service returned failure
            Status.SUCCESS if land service returned success
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
            self.logger.info(f"Land service call result: {response.message}\n")
            if response.success:
                return Status.SUCCESS
            else:
                return Status.FAILURE
        except Exception as e:
            self.logger.error(f'Service call failed: {str(e)}')
            return Status.FAILURE

    def terminate(self, new_status) -> None:
        """This is called whenever your node switches to a non running state (SUCCESS, FAILURE, or INVALID)

        Args:
            new_status: stateus we terminate  the node with 
        """
        self.logger.info(f"Terminating with new status: {new_status}")

        self.future = None
        