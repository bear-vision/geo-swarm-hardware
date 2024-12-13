import py_trees

class DummyBlackboardReader(py_trees.behaviour.Behaviour):
    def __init__(self, name="BlackboardReader"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client() # create blackboard client
        self.blackboard.register_key("drone/position/x", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/position/y", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/position/z", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/orientation/yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/valid/xy_valid", access=py_trees.common.Access.READ)
        self.blackboard.register_key("drone/valid/z_valid", access=py_trees.common.Access.READ)


        self.blackboard.register_key("paint/found", access=py_trees.common.Access.READ)
        self.blackboard.register_key("paint_target", access=py_trees.common.Access.READ)
        self.blackboard.register_key("visited_paint_positions", access=py_trees.common.Access.READ)

    def update(self):
        """Called on each tick, will print the relevant VehicleLocalPosition values
        """
        # try:
        # print(f"Valid: xy={self.blackboard.drone.valid.xy_valid}, "
        #         f"z={self.blackboard.drone.valid.z_valid}")
        self.logger.info(f"Position: x={self.blackboard.drone.position.x:.2f},\n y={self.blackboard.drone.position.y:.2f}, \n z={self.blackboard.drone.position.z:.2f}\n")
        # print(f"Yaw: {self.blackboard.drone.orientation.yaw:.2f}")
        # self.logger.info(f"Paint found: {self.blackboard.paint.found}\n")
        self.logger.info(f"Paint target: {self.blackboard.paint_target}\n")
        self.logger.info(f"Visited paint positions: {self.blackboard.visited_paint_positions}\n")
        return py_trees.common.Status.SUCCESS
        # except AttributeError:
        #     print("Waiting for blackboard data...")
        #     return py_trees.common.Status.RUNNING