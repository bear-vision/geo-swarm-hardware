import py_trees
from py_trees.common import Status

class DetectPaintBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, behaviour_name):
        super().__init__(behaviour_name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("paint/found", access =  py_trees.common.Access.READ)     
        
    
    def update(self) -> Status:
        """Check for paint.
        
        Returns:
            Status.SUCCESS: when paint is found by perception node
            Status.RUNNING: no paint detected yet
        """
        if self.blackboard.paint.found:
            self.logger.info(f"Paint detected")
            return Status.SUCCESS
        else:
            self.logger.debug(f"No paint.")
            return Status.RUNNING
    
