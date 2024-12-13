"""NEED TO TEST"""

import py_trees
from py_trees.common import Status

class TurnOffSprayerBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, behaviour_name, service_name, service_type):
        super().__init__(behaviour_name)
        self.sprayer_client = None
        self.service_name = service_name
        self.service_type = service_type
        self.future = None
        
    def setup(self, **kwargs) -> None:
        """Sets up the sprayer service.
        
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
        self.sprayer_client = self.node.create_client(self.service_type, self.service_name)
        while not self.sprayer_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Waiting for the sprayer service...')     
            
            
    def initialise(self) -> None:
        """Creates service request."""
        request = self.service_type.Request() # empty
        self.future = self.sprayer_client.call_async(request)
        
    def update(self) -> Status:
        """Check service until it is completed
        
        Returns:
            Status.FAILURE if there is no service client yet, or service call fails
            Status.SUCCESS if sprayer client returns true
            Status.RUNNING if we are waiting for the service response
        """
        self.logger.debug(f"{self.qualified_name}.update()")
        
        if self.future is None:
            self.logger.error("No sprayer")
            return Status.FAILURE
        
        if not self.future.done():
            return Status.RUNNING
        
        try:
            response = self.future.result()
            if response.success:
                return Status.SUCCESS
            else:
                self.logger.error(f'Sprayer service was unsuccessfull')
                return Status.FAILURE
        except Exception as e:
            self.logger.error(f'Sprayer service call failed: {str(e)}')
            return Status.FAILURE
