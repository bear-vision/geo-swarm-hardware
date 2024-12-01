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
Last modified: 11/30/2024
"""

from py_trees.common import Status
import rclpy
import py_trees
import py_trees_ros
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from std_srvs.srv import Trigger
from custom_srv_interfaces.srv import PathPlannerPaint, PathPlannerUp, PathPlannerSpin
from sensor_msgs.msg import Image
from laptop_master.behaviours.local_position_2BB import *
from laptop_master.behaviours.perception_2BB import *
from laptop_master.behaviours.get_waypoints_from_service import GetWaypointsFromService
from laptop_master.behaviours.sprayer import SprayerBehaviour
import sys
 
        
def create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name="Test",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    gather_data = py_trees.composites.Sequence(name="Gather Data", memory=True)
    localPosition2BB =  vehicle_local_position_to_blackboard()
    perception2BB = perception_to_blackboard()
    gather_data.add_children([localPosition2BB, perception2BB])
    
    navigate_to_tower_sequence = py_trees.composites.Sequence(name="Navigate To Tower", memory=True)
    get_waypoints_to_tower = GetWaypointsFromService(
        behaviour_name="Get Waypoints To Tower", 
        service_type=PathPlannerSpin, 
        service_name='plan_path_spin',
        blackboard_waypoint_key="to_tower"
    )

    get_waypoints_around_tower = GetWaypointsFromService(
        behaviour_name="Get Waypoints Around Tower", 
        service_type=PathPlannerSpin, 
        service_name='plan_path_spin',
        blackboard_waypoint_key="around_tower"
    ) 
    
    get_waypoints_to_paint = GetWaypointsFromService(
        behaviour_name="Get Waypoints To Paint", 
        service_type=PathPlannerPaint, 
        service_name='plan_path_paint',
        blackboard_waypoint_key="to_paint"
    )
    get_waypoints_up = GetWaypointsFromService(
        behaviour_name="Get Waypoints Up", 
        service_type=PathPlannerUp, 
        service_name='plan_path_up',
        blackboard_waypoint_key="up"
    )


    actuate_sprayer = SprayerBehaviour(behaviour_name="Actuate Sprayer", service_type=Trigger, service_name='/rpi_master/rpi_sprayer_on')
    turn_off_sprayer = SprayerBehaviour(behaviour_name="Turn Off Sprayer", service_type=Trigger, service_name='/rpi_master/rpi_sprayer_off')



    tasks = py_trees.composites.Sequence(name="Tasks", memory=True)
    idle = py_trees.behaviours.Running(name="Idle")
    # flipper = py_trees.behaviours.Periodic(name="Flip Eggs", n=2)

    root.add_children([gather_data, tasks])
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