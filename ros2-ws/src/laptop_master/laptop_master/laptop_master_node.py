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
import sys

'''
PX4 requires that the vehicle is already receiving OffboardControlMode messages 
before it will arm in offboard mode, or before it will switch to offboard mode when flying. 
In addition, PX4 will switch out of offboard mode if the stream rate of OffboardControlMode 
messages drops below approximately 2Hz.
'''

# class DroneBaseBehaviour(py_trees.behaviour.Behaviour):
#     '''Common node setup to avoid code duplication'''
#     def __init__(self, name):
#         # Call parent class constructor with name
#         py_trees.behaviour.Behaviour.__init__(self, name)
#         self.node = None
        
#     def setup(self, **kwargs):
#         """
#         Setup the publishers:
#             - OffboardControlMode: informs PX4 the type of offboard control being used (we are interested in position only)
#             - TrajectorySetpoint: provides the position setpoint that can be dynamically updated
#             - VehicleCommand: like the name suggests, sets commands (we are interested in setting vehicle to offboard mode, arm, disarm)

#         Setup services:
#             - PathPlannerSpin: creates waypoints towards the tower if out of range, or around tower if in range
#             - PathPlannerPaint: creates waypoints towards the paint (move closer to the tower in the same plane of the circle trajectory)
#             - PathPlannerUp: creates waypoints to move drone up by a certain height
        
#         Args:
#             **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

#         Raises:
#             :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
#         """
#         # Get node from the tree
#         self.logger.debug("{}.setup()".format(self.qualified_name))
#         try:
#             self.node = kwargs['node']
#         except KeyError as e:
#             error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
#             raise KeyError(error_message) from e  # 'direct cause' traceability

#         # Set publishers
#         self.offboard_control_pub = self.node.create_publisher(
#             msg_type = OffboardControlMode,
#             topic = "/fmu/in/offboard_control_mode",
#             qos_profile = py_trees_ros.utilities.qos_profile_latched()
#         )
#         self.trajectory_setpoint_pub = self.node.create_publisher(
#             msg_type = TrajectorySetpoint,
#             topic = "/fmu/in/trajectory_setpoint",
#             qos_profile = py_trees_ros.utilities.qos_profile_latched()
#         )
#         self.vehicle_command_pub = self.node.create_publisher(
#             msg_type = VehicleCommand,
#             topic = "/fmu/in/vehicle_command",
#             qos_profile = py_trees_ros.utilities.qos_profile_latched()
#         )
        
#         # Set up service client
#         self.go_around_tower_client = self.node.create_client(PathPlannerSpin, 'plan_path_spin')
#         while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for the plan_path_spin service...')
            
#         self.go_to_paint_client = self.node.create_service(PathPlannerPaint, 'plan_path_paint')
#         while not self.paint_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for the plan_path_paint service...')
                    
#         self.go_up_client = self.node.create_service(PathPlannerUp, 'plan_path_up')
#         while not self.path_up_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for the plan_path_up service...')
            
#     def publish_offboard_control(self):
#         '''
#         Publish the offboard control node.
        
#         Note:
#             - We only set position and altitude controls.
#         '''
#         msg = OffboardControlMode()
#         msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000.0)
#         msg.position = True
#         msg.altitude = True
#         self.offboard_control_pub.publish(msg
#                                           )
            
#     def publish_trajectory_setpoint(self, x:float, y:float, z:float, yaw:float):
#         '''
#         Publish a trajectory setpoint (in NED frame). This is the input to the PID position controller
#         on the PX4.
        
#         Note:
#             - We only care about position and yaw.
#         '''
#         msg = TrajectorySetpoint()
#         msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000.0)
#         msg.position = [x, y, z]
#         msg.yaw = yaw
#         self.trajectory_setpoint_pub.publish(msg)
    
#     def publish_vehicle_command(self, cmd, param1: float = 0.0, param2: float = 0.0):
#         '''
#         Publish vehicle commands.
        
#         Args: 
#             - cmd:    Command code (matches VehicleCommand and MAVLink MAV_CMD codes -- see px4_msgs.msg)
#             - param1: Command parameter 1
#             - param2: Command parameter 2
#         '''
#         msg = VehicleCommand()
#         msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000.0)
#         msg.param1 = param1
#         msg.param2 = param2
#         msg.command = cmd
#         msg.target_system = 1
#         msg.target_component = 1
#         msg.source_system = 1
#         msg.source_component = 1
#         msg.from_external = True
#         self.vehicle_command_pub.publish(msg)
        
#     def disarm(self):
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    
#     def arm(self):
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            
        
def create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name="Tutorial One",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    
    # This will save the entire message
    localPosition2BB =  vehicle_local_position_to_blackboard()
    dummy_blackboard_reader = DummyBlackboardReader()

    priorities = py_trees.composites.Selector(name="Tasks", memory=False)
    idle = py_trees.behaviours.Running(name="Idle")
    flipper = py_trees.behaviours.Periodic(name="Flip Eggs", n=2)

    root.add_child(topics2bb)
    topics2bb.add_child(localPosition2BB)
    # root.add_child(priorities)
    root.add_child(dummy_blackboard_reader)
    # priorities.add_child(flipper)
    # priorities.add_child(idle)

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
