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

from py_trees.common import Status
import rclpy
import py_trees
import py_trees_ros
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from std_srvs.srv import Trigger
from custom_interfaces.srv import PathPlannerPaint, PathPlannerUp, PathPlannerSpin
from sensor_msgs.msg import Image
from laptop_master.behaviours.local_position_2BB import *
from laptop_master.behaviours.perception_2BB import *
from laptop_master.behaviours.get_waypoints_tower import GetWaypointsTower
from laptop_master.behaviours.get_waypoints_up import GetWaypointsUp
from laptop_master.behaviours.sprayer import SprayerBehaviour
from laptop_master.behaviours.follow_waypoints import FollowWaypoints
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import sys
 
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

    gather_data = py_trees.composites.Sequence(name="Gather Data", memory=True)
    localPosition2BB =  vehicle_local_position_to_blackboard(qos_profile)
    perception2BB = perception_to_blackboard()
    gather_data.add_children([localPosition2BB, perception2BB])
    
    tasks = py_trees.composites.Sequence(name="Tasks", memory=True)
    
    check_height = py_trees.composites.Selector(name="Check Height", memory=False)
    land = py_trees.behaviours.Running(name="Success!") # TODO - idles now, need to implement land behaviour with service
    move_up_sequence = py_trees.composites.Sequence(name="Move Up Sequence", memory=True)
    get_waypoints_up = GetWaypointsUp(
        behaviour_name="Get Waypoints Up",
        blackboard_waypoint_key="up"
    )
    follow_waypoints_up = FollowWaypoints(
        behaviour_name="Follow Waypoints Up",
        blackboard_waypoint_key="up"
    )
    move_up_sequence.add_children([get_waypoints_up, follow_waypoints_up])
    
    def drone_above_threshold_height(blackboard: py_trees.blackboard.Blackboard) -> bool:
        height_threshold_ned = -10
        if blackboard.drone.position.z <= height_threshold_ned:
            return True
        return False
    
    height_above_threshold = py_trees.decorators.EternalGuard(
        name="Above Height?",
        condition=drone_above_threshold_height,
        blackboard_keys={"/drone/position/z"},
        child=land
    )
    
    check_height.add_children([height_above_threshold, move_up_sequence])
    
    
    
    navigate_to_tower_sequence = py_trees.composites.Sequence(name="Navigate To Tower", memory=True)
    get_waypoints_to_tower = GetWaypointsTower(
        behaviour_name="Get Waypoints To Tower", 
        service_type=PathPlannerSpin, 
        service_name='plan_path_spin',
        blackboard_waypoint_key="to_tower"
    )
    follow_waypoints_to_tower = FollowWaypoints(
        behaviour_name="Follow Waypoints To Tower",
        blackboard_waypoint_key="to_tower"
    )
    navigate_to_tower_sequence.add_children([get_waypoints_to_tower, follow_waypoints_to_tower])
    
    circle_tower = py_trees.composites.Sequence(name="Circle Tower", memory=True)
    get_waypoints_around_tower = GetWaypointsTower(
        behaviour_name="Get Waypoints Around Tower",
        service_type=PathPlannerSpin,
        service_name="plan_path_spin",
        blackboard_waypoint_key="around_tower"
    )
    follow_waypoints_around_tower=FollowWaypoints(
        behaviour_name="Follow Waypoints Around Tower",
        blackboard_waypoint_key="around_tower"
    )
    circle_tower.add_children([get_waypoints_around_tower, follow_waypoints_around_tower])
        
    # idle = py_trees.behaviours.Running(name="Success!")
    # tasks.add_children([check_height, navigate_to_tower_sequence, circle_tower, idle])
    tasks.add_children([check_height, circle_tower])

    

    # get_waypoints_around_tower = GetWaypointsTower(
    #     behaviour_name="Get Waypoints Around Tower", 
    #     service_type=PathPlannerSpin, 
    #     service_name='plan_path_spin',
    #     blackboard_waypoint_key="waypoints/around_tower"
    # ) 
    # actuate_sprayer = SprayerBehaviour(behaviour_name="Actuate Sprayer", service_type=Trigger, service_name='/rpi_master/rpi_sprayer_on')
    # turn_off_sprayer = SprayerBehaviour(behaviour_name="Turn Off Sprayer", service_type=Trigger, service_name='/rpi_master/rpi_sprayer_off')

    # flipper = py_trees.behaviours.Periodic(name="Flip Eggs", n=2)

    root.add_children([gather_data, tasks])
    

    return root


def main(args=None):
    print('Hi from laptop_master.')
    
    # FOR DEBUGGING
    # py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="laptop_master_node", timeout=15.0)
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

    # tree.visitors.append(py_trees.visitors.DebugVisitor())
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
