import py_trees
import py_trees_ros
import inspect
import functools
from py_trees.common import Status
from py_trees.decorators import Decorator
from py_trees.behaviour import Behaviour


"""
Run the subtree the starting the first time a condition is met. Will keep running after the first flip, even when the condiiton is no longer met.
"""
class RunOnCondition(py_trees.decorators.Decorator):
    def __init__(self, name: str, child: Behaviour, condition_fn, blackboard_keys = None):
        super().__init__(name=name, child=child)

        self.blackboard = self.attach_blackboard_client(self.name)
        self.blackboard_keys = blackboard_keys
        if self.blackboard_keys is None:
            self.blackboard_keys = []
        for key in self.blackboard_keys:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        
        condition_signature = inspect.signature(condition_fn)
        if "blackboard" in [p.name for p in condition_signature.parameters.values()]:
            self.condition_fn = functools.partial(condition_fn, self.blackboard)
        else:
            self.condition_fn = condition_fn

        self.condition_met = False

    def initialise(self):
        self.condition_met = False
        # self.decorated.initialise()  # Initialize the child when condition is met
        # self.logger.info(f"Condition met!")

    def tick(self):
        """
        Conditionally manage the child.

        Yields:
            a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)

        if not self.condition_met:
            
            self.condition_met = self.condition_fn()
            self.stop(py_trees.common.Status.FAILURE)
            yield self
        else:
            for node in super().tick():
                yield node


    def update(self):
        return self.decorated.status

    """
    def update(self):
        status = self.decorated.status

        if not self.condition_met:
            if self.condition_fn():
                self.condition_met = True
                self.logger.info(f"Condition met!")
                self.decorated.initialise()
            else:
                self.terminate(py_trees.common.Status.FAILURE)
                return py_trees.common.Status.FAILURE

        
        return self.decorated.update()
    """

    def terminate(self, new_status):
        self.decorated.terminate(new_status)
        self.condition_met = False
