import py_trees
from py_trees.common import Status

class BBLoggerBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, behaviour_name, blackboard_keys):
        super().__init__(behaviour_name)

        self.blackboard_Keys = blackboard_keys
        self.blackboard = self.attach_blackboard_client() 

        for key in blackboard_keys:
            self.blackboard.register_key(key, access=py_trees.common.Access.READ)

        
    def setup(self, **kwargs) -> None:
        return
    
    def initialise(self):
        return
        
    def update(self) -> Status:
        """To get the last drone pose on the blackboard.

        Returns:
            Status.FAILURE the blackboard isn't set up?
            Status.SUCCESS if we can read and log from the blackboard.
        """
        try:
            x, y, z = self.blackboard.drone.position.x, self.blackboard.drone.position.y, self.blackboard.drone.position.z
            yaw = self.blackboard.drone.orientation.yaw

            self.logger.info(f"Latest drone pose on bt blackboard: \n - (x, y, z) = {(x, y, z)}, yaw = {yaw}\n")

            return Status.SUCCESS
        except Exception as e:
            self.logger.info(f"Failure in retrieving drone local pose from blackboard: {e}\n")
        finally:
            pass
        return Status.FAILURE


    def terminate(self, new_status) -> None:
        """This is called whenever your node switches to a non running state (SUCCESS, FAILURE, or INVALID)

        Args:
            new_status: status we terminate the node with 
        """
        return