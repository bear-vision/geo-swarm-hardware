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
from custom_interfaces.srv import PathPlannerPaint, PathPlannerUp, PathPlannerSpin, PathPlannerTower
from sensor_msgs.msg import Image
from laptop_master.behaviours.local_position_2BB import *
from laptop_master.behaviours.perception_2BB import *
from laptop_master.behaviours.dummy_blackboard_reader import *
# from laptop_master.behaviours.bb_logger import *
from laptop_master.behaviours.land_drone import *
from laptop_master.behaviours.get_waypoints_circle import GetWaypointsCircle
from laptop_master.behaviours.get_waypoints_tower import GetWaypointsTower
from laptop_master.behaviours.get_waypoints_up import GetWaypointsUp
from laptop_master.behaviours.get_waypoints_down_one_level import GetWaypointsDownOneLevel
from laptop_master.behaviours.get_waypoints_one_paint_blob import GetWaypointsOnePaintBlob
from laptop_master.behaviours.sprayer import SprayerBehaviour
from laptop_master.behaviours.follow_waypoints import FollowWaypoints
from laptop_master.behaviours.follow_next_waypoint import FollowNextWaypoint
# from laptop_master.behaviours.follow_circle_waypoints import FollowCircleWaypoints
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from laptop_master.decorators.repeat_until_condition import *
from laptop_master.decorators.run_on_condition import *

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
    # bb_logger = BBLogger(
    #     behaviour_name="Blackboard Logger",
    #     blackboard_keys=['/drone/position/x','/drone/position/y', '/drone/position/z', 'drone/orientation/yaw']
    # )
    dummy_bb_reader = DummyBlackboardReader()
    gather_data.add_children([localPosition2BB, perception2BB, dummy_bb_reader])

    move_up_sequence = py_trees.composites.Sequence(name="Move Up Sequence", memory=True)
    get_waypoints_up = GetWaypointsUp(
        behaviour_name="Get Waypoints Up",
        blackboard_waypoint_key="up",
        height_diff = 3.0
    )
    follow_waypoints_up = FollowWaypoints(
        behaviour_name="Follow Waypoints Up",
        blackboard_waypoint_key="up"
    )
    move_up_sequence.add_children([get_waypoints_up, follow_waypoints_up])
    move_up_oneshot = py_trees.decorators.OneShot("Move Up Oneshot", child = move_up_sequence, policy = py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION)

    approach_tower_sequence = py_trees.composites.Sequence(name="Approach Tower", memory=True)
    get_waypoints_approach_tower = GetWaypointsTower(
        behaviour_name="Get Waypoints to Approach Tower",
        service_type=PathPlannerTower,
        service_name="plan_path_tower",
        blackboard_waypoint_key="approach_tower"
    )
    follow_waypoints_approach_tower=FollowWaypoints(
        behaviour_name="Follow Waypoints to Approach Tower",
        blackboard_waypoint_key="approach_tower"
    )
    approach_tower_sequence.add_children([get_waypoints_approach_tower, follow_waypoints_approach_tower])
    approach_tower_oneshot = py_trees.decorators.OneShot("Approach Tower Oneshot", child = approach_tower_sequence, policy = py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION)

    tasks = py_trees.composites.Sequence(name="Tasks", memory=True)
    
    land_drone_behavior = LandDrone(behaviour_name = 'Land Drone')
    land_oneshot = py_trees.decorators.OneShot("Land Oneshot", child = land_drone_behavior, policy = py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION)
    
    move_down_sequence = py_trees.composites.Sequence(name="Move Down Sequence", memory=True)
    get_waypoints_down_one_level = GetWaypointsDownOneLevel(
        behaviour_name="Get Waypoints Down",
        blackboard_waypoint_key="down",
        height_diff = -1.0
    )
    follow_waypoints_down_one_level = FollowWaypoints(
        behaviour_name="Follow Waypoints Down",
        blackboard_waypoint_key="down"
    )

    def drone_below_threshold_height(blackboard: py_trees.blackboard.Blackboard) -> bool:
        height_threshold_ned = -1.5
        if blackboard.drone.position.z >= height_threshold_ned:
            return True
        return False

    circle_tower = py_trees.composites.Sequence(name="Circle Tower", memory=True)
    get_waypoints_around_tower = GetWaypointsCircle(
        behaviour_name="Get Waypoints for Circling Tower",
        service_type=PathPlannerSpin,
        service_name="plan_path_spin",
        blackboard_waypoint_key="around_tower"
    )
    # follow_waypoints_around_tower=FollowCircleWaypoints(
    #     behaviour_name="Follow Waypoints Around Tower",
    #     blackboard_waypoint_key="around_tower"
    # )
    # circle_tower.add_children([get_waypoints_around_tower, follow_waypoints_around_tower, get_waypoints_down_one_level, follow_waypoints_down_one_level])
    
    repeat_circle_until_below_threshold = RepeatUntilCondition(
        name = "Repeat Circle until Below Threshold",
        child = circle_tower,
        condition_fn = drone_below_threshold_height,
        blackboard_keys = ['/drone/position/z']
        )
        
    clean_sequence = py_trees.composites.Sequence(
        name="Clean Sequence",
        memory="False"
    )
    get_waypoints_to_paint = GetWaypointsOnePaintBlob(
        behaviour_name="Get Waypoints To Paint",
        service_type=PathPlannerPaint,
        service_name="plan_path_paint",
        blackboard_waypoint_key="to_paint"
    )
    follow_waypoints_to_paint = FollowWaypoints(
        behaviour_name="Follow Waypoints To Paint",
        blackboard_waypoint_key="to_paint"
    )
    # TODO - test with RPI, or write dummy services for sprayer.
    # actuate_sprayer = SprayerBehaviour(behaviour_name="Actuate Sprayer", service_type=Trigger, service_name='/rpi_master/rpi_sprayer_on')
    # turn_off_sprayer = SprayerBehaviour(behaviour_name="Turn Off Sprayer", service_type=Trigger, service_name='/rpi_master/rpi_sprayer_off')
    
    return_to_radius_sequence = py_trees.composites.Sequence(name="Approach Tower", memory=True)
    get_waypoints_to_radius = GetWaypointsTower(
        behaviour_name="Get Waypoints to Approach Tower",
        service_type=PathPlannerTower,
        service_name="plan_path_tower",
        blackboard_waypoint_key="approach_tower"
    )
    follow_waypoints_to_radius=FollowWaypoints(
        behaviour_name="Follow Waypoints to Approach Tower",
        blackboard_waypoint_key="approach_tower"
    )
    return_to_radius_sequence.add_children([get_waypoints_to_radius, follow_waypoints_to_radius])
    

    #example for going to paint and coming back from paint
    clean_sequence.add_children([get_waypoints_to_paint, follow_waypoints_to_paint, return_to_radius_sequence])
    
    def check_for_paint_detected(blackboard: py_trees.blackboard.Blackboard):
        return blackboard.paint.found
    
    paint_detected = py_trees.decorators.EternalGuard(
        name="Paint Detected?",
        condition=check_for_paint_detected,
        blackboard_keys=["paint/found"],
        child=clean_sequence
    )

    # paint_detected = RunOnCondition(
    #     name="Paint Detected?",
    #     condition_fn=check_for_paint_detected,
    #     blackboard_keys=["paint/found"],
    #     child=clean_sequence
    # )
    
    inspect_and_go_next_waypoint = py_trees.composites.Selector(
        name="Inspect And Go To Next Waypoint",
        memory=True #TODO: check
    )

    follow_next_waypoint = FollowNextWaypoint(
        behaviour_name="Follow Next Waypoint",
        blackboard_waypoint_key="around_tower"
    )

    #replace with follow_next_waypoint
    inspect_and_go_next_waypoint.add_children([paint_detected, follow_next_waypoint])
    
    def is_drone_done_circling(blackboard: py_trees.blackboard.Blackboard):
        # return blackboard.finished_circle_layer
        if len(blackboard.waypoints['around_tower'].poses) == 0:
            logger.warn(f"Waypoints length is 0")
            return False
        return blackboard.waypoint_index >= len(blackboard.waypoints['around_tower'].poses)
    
    repeat_until_all_waypoints_inspected = RepeatUntilCondition(
        name="Repeat until All Waypoints Inspected",
        child=inspect_and_go_next_waypoint,
        condition_fn=is_drone_done_circling,
        # blackboard_keys= ["finished_circle_layer"]
        blackboard_keys = ['waypoints', 'waypoint_index']
    )
    
    circle_tower.add_children([get_waypoints_around_tower, repeat_until_all_waypoints_inspected, get_waypoints_down_one_level, follow_waypoints_down_one_level])
    
    
    idle = py_trees.behaviours.Running(name="Success!")
    tasks.add_children([move_up_oneshot, approach_tower_oneshot, repeat_circle_until_below_threshold, land_oneshot, idle])
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

    # Create a MultiThreadedExecutor to handle both nodes concurrently
    # executor = MultiThreadedExecutor()
    # executor.add_node(tree.node)

    # tree.visitors.append(py_trees.visitors.DebugVisitor())
    # tree.tick_tock(period_ms=100.0) # 10 Hz

    tree.tick_tock(period_ms=500.0)
    try:
        # executor.spin()
        rclpy.spin(tree.node)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # executor.shutdown()
        # tree.node.destroy_node()
        tree.shutdown()
        rclpy.try_shutdown()
        
    

if __name__ == '__main__':
    main()
