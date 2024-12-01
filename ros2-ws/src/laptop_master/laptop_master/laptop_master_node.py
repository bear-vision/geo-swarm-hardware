"""
A behavior tree implements the logic of the drone. Each action (or behavior) is
defined in a class. 

Behavior tree tutorial here: 
https://py-trees-ros-tutorials.readthedocs.io/en/release-2.1.x/tutorials.html#tutorials

A node is instantiated to allow communication with other ROS nodes. For example,
we need to call services from path_planning node, subscribe to /detected_paint from
the perception node, send px4 messages to control the drone flight, and communicate
with the rpi to actuate the sprayers.

By: Alexandra Zhang Jiang 
Last modified: 11/24/2024
"""

from py_trees.common import Status
import rclpy
import py_trees
import py_trees_ros
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from custom_srv_interfaces.srv import PathPlannerPaint, PathPlannerUp, PathPlannerSpin
from sensor_msgs.msg import Image
from laptop_master.behaviours.local_position_2BB import *
from laptop_master.behaviours.dummy_blackboard_reader import *
from laptop_master.behaviours.get_waypoints_from_service import *
import sys
 
        
def create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name="Test Service",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    
    # This will save the entire message
    localPosition2BB =  vehicle_local_position_to_blackboard()
    dummy_blackboard_reader = DummyBlackboardReader()
    
    # the service checks for radius away from tower, returns a linear path if too far, otherwise returns a circular path
    get_waypoints_to_tower = GetWaypointsFromService(service_type=PathPlannerSpin, service_name='plan_path_spin')
    get_waypoints_around_tower = GetWaypointsFromService(service_type=PathPlannerSpin, service_name='plan_path_spin') 

    tasks = py_trees.composites.Sequence(name="Tasks", memory=True)
    idle = py_trees.behaviours.Running(name="Idle")
    # flipper = py_trees.behaviours.Periodic(name="Flip Eggs", n=2)

    root.add_children([topics2bb, tasks])
    topics2bb.add_child(localPosition2BB)
    tasks.add_children([get_waypoints_to_tower, idle])

    return root


def main(args=None):
    '''
    https://github.com/splintered-reality/py_trees_ros_tutorials/blob/devel/py_trees_ros_tutorials/one_data_gathering.py
    '''
    print('Hi from laptop_master.')
    
    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="foo", timeout=15.0)
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

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
        
    

if __name__ == '__main__':
    main()
