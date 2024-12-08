import py_trees
import py_trees_ros
import inspect
import functools
from py_trees.common import Status
from py_trees.decorators import Decorator
from py_trees.behaviour import Behaviour

class RepeatUntilCondition(py_trees.decorators.Decorator):
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

    def update(self):
        status = self.decorated.status
        if status == py_trees.common.Status.SUCCESS:
            if self.condition_fn():
                return py_trees.common.Status.SUCCESS

            self.decorated.stop(py_trees.common.Status.INVALID)
            self.decorated.initialise()
            return py_trees.common.Status.RUNNING

        return status
