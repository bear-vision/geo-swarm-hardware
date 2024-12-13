import py_trees
from py_trees.common import Status

class DetectPaintBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, behaviour_name):
        super().__init__(behaviour_name)
        
    def setup(self, **kwargs):
        return super().setup(**kwargs)
    
    def initialise(self):
        return super().initialise()
    
    def update(self):
        return super().update()
    
    def terminate(self, new_status):
        return super().terminate(new_status)
    