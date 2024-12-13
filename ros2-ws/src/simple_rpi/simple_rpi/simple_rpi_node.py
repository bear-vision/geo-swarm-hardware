"""
A behavior tree implements the logic of the drone. Each action (or behavior) is
defined in a class. 

Behavior tree tutorial here: 
https://py-trees-ros-tutorials.readthedocs.io/en/release-2.1.x/tutorials.html#tutorials

Tutorial about how to set up py-trees behaviours with ROS subs, pubs, actions, and services:
https://arvp.org/wp-content/uploads/development-documents/ARVP-Mission-Planner-Documentation.pdf

By: Alexandra Zhang Jiang 
Last modified: 12/01/2024
"""

import rclpy
import py_trees
import py_trees_ros
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import MultiThreadedExecutor
import sys
from std_srvs.srv import Trigger

from simple_rpi.behaviours.detect_paint import DetectPaintBehaviour
from simple_rpi.behaviours.turn_on_sprayer import TurnOnSprayerBehaviour
from simple_rpi.behaviours.turn_off_sprayer import TurnOffSprayerBehaviour
from simple_rpi.behaviours.bb_logger import BBLoggerBehaviour
from simple_rpi.behaviours.perception_2BB import *

def custom_qos():
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
    )
    return qos_profile
        
        
def create_root() -> py_trees.behaviour.Behaviour:
    # TODO make sure that when no data available, you don't try to get waypoints
    root = py_trees.composites.Parallel(
        name="Test",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )
    
    qos_profile = custom_qos()

    gather_data = py_trees.composites.Sequence(name="Gather Data", memory = True)
    tasks = py_trees.composites.Sequence(name="Tasks", memory = True)
    
    perception2BB = perception_to_blackboard()
    bb_logger = BBLoggerBehaviour(
        behaviour_name="Blackboard Logger",
        blackboard_keys=['/drone/position/x','/drone/position/y', '/drone/position/z', 'drone/orientation/yaw']
    )
    gather_data.add_children([perception2BB, bb_logger])

    detect_paint = DetectPaintBehaviour(behaviour_name="Detect Paint")
    actuate_sprayer = TurnOnSprayerBehaviour(behaviour_name="Actuate Sprayer", service_type=Trigger, service_name='rpi_sprayer_on')
    turn_off_sprayer = TurnOffSprayerBehaviour(behaviour_name="Turn Off Sprayer", service_type=Trigger, service_name='rpi_sprayer_off')    
    tasks.add_children([detect_paint, actuate_sprayer, turn_off_sprayer])
    root.add_children([gather_data, tasks])

    return root


def main(args=None):
    print('Hi from simple_rpi_node.')
    
    # FOR DEBUGGING
    # py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="simple_rpi_node", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        py_trees.console.logerror(py_trees.console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + py_trees.console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        py_trees.console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=500.0)
    try:
        rclpy.spin(tree.node)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
        
    

if __name__ == '__main__':
    main()
