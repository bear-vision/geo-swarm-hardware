import py_trees
import py_trees_ros

class DummyBlackboardReader(py_trees.behaviour.Behaviour):
    def __init__(self, name="BlackboardReader"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client() # create blackboard client
        self.blackboard.register_key("position/x", access=py_trees.common.Access.READ)
        self.blackboard.register_key("position/y", access=py_trees.common.Access.READ)
        self.blackboard.register_key("position/z", access=py_trees.common.Access.READ)
        self.blackboard.register_key("orientation/yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key("valid/xy_valid", access=py_trees.common.Access.READ)
        self.blackboard.register_key("valid/z_valid", access=py_trees.common.Access.READ)
        
    def update(self):
        """Called on each tick, will print the relevant VehicleLocalPosition values
        """
        # try:
        print(f"Valid: xy={self.blackboard.valid.xy_valid}, "
                f"z={self.blackboard.valid.z_valid}")
        print(f"Position: x={self.blackboard.position.x:.2f}, "
                f"y={self.blackboard.position.y:.2f}, "
                f"z={self.blackboard.position.z:.2f}")
        print(f"Yaw: {self.blackboard.orientation.yaw:.2f}")
        return py_trees.common.Status.SUCCESS
        # except AttributeError:
        #     print("Waiting for blackboard data...")
        #     return py_trees.common.Status.RUNNING