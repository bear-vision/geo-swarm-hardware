import py_trees
from py_trees.common import Status

class Land(py_trees.behaviour.Behaviour):
    def __init__(self, behaviour_name):
        super().__init__(behaviour_name)
        self.land_client = None
        self.service_name = '/rpi_master/land_drone'