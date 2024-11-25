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
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from custom_srv_interfaces.srv import PathPlannerPaint, PathPlannerUp, PathPlannerSpin
from sensor_msgs.msg import Image

'''
PX4 requires that the vehicle is already receiving OffboardControlMode messages 
before it will arm in offboard mode, or before it will switch to offboard mode when flying. 
In addition, PX4 will switch out of offboard mode if the stream rate of OffboardControlMode 
messages drops below approximately 2Hz.
'''

class DroneBaseBehaviour(py_trees.behaviour.Behaviour):
    '''Common node setup to avoid code duplication'''
    def __init__(self):
        super().__init__(name = "Drone Base Bahaviour")
        
    def setup(self, **kwargs):
        """
        Setup the publishers:
            - OffboardControlMode: informs PX4 the type of offboard control being used (we are interested in position only)
            - TrajectorySetpoint: provides the position setpoint that can be dynamically updated
            - VehicleCommand: like the name suggests, sets commands (we are interested in setting vehicle to offboard mode, arm, disarm)

        Setup services:
            - PathPlannerSpin: creates waypoints towards the tower if out of range, or around tower if in range
            - PathPlannerPaint: creates waypoints towards the paint (move closer to the tower in the same plane of the circle trajectory)
            - PathPlannerUp: creates waypoints to move drone up by a certain height
        
        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        # Get node from the tree
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Set publishers
        self.offboard_control_pub = self.node.create_publisher(
            msg_type = OffboardControlMode,
            topic = "/fmu/in/offboard_control_mode",
            qos_profile = py_trees_ros.utilities.qos_profile_latched()
        )
        self.trajectory_setpoint_pub = self.node.create_publisher(
            msg_type = TrajectorySetpoint,
            topic = "/fmu/in/trajectory_setpoint",
            qos_profile = py_trees_ros.utilities.qos_profile_latched()
        )
        self.vehicle_command_pub = self.node.create_publisher(
            msg_type = VehicleCommand,
            topic = "/fmu/in/vehicle_command",
            qos_profile = py_trees_ros.utilities.qos_profile_latched()
        )
        
        # Set up service client
        self.go_around_tower_client = self.node.create_client(PathPlannerSpin, 'plan_path_spin')
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the plan_path_spin service...')
            
        self.go_to_paint_client = self.node.create_service(PathPlannerPaint, 'plan_path_paint')
        while not self.paint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the plan_path_paint service...')
                    
        self.go_up_client = self.node.create_service(PathPlannerUp, 'plan_path_up')
        while not self.path_up_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the plan_path_up service...')
            
    def publish_offboard_control(self):
        '''
        Publish the offboard control node.
        
        Note:
            - We only set position and altitude controls.
        '''
        msg = OffboardControlMode()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000.0)
        msg.position = True
        msg.altitude = True
        self.offboard_control_pub.publish(msg
                                          )
            
    def publish_trajectory_setpoint(self, x:float, y:float, z:float, yaw:float):
        '''
        Publish a trajectory setpoint (in NED frame). This is the input to the PID position controller
        on the PX4.
        
        Note:
            - We only care about position and yaw.
        '''
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000.0)
        msg.position = [x, y, z]
        msg.yaw = yaw
        self.trajectory_setpoint_pub.publish(msg)
    
    def publish_vehicle_command(self, cmd, param1: float = 0.0, param2: float = 0.0):
        '''
        Publish vehicle commands.
        
        Args: 
            - cmd:    Command code (matches VehicleCommand and MAVLink MAV_CMD codes -- see px4_msgs.msg)
            - param1: Command parameter 1
            - param2: Command parameter 2
        '''
        msg = VehicleCommand()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000.0)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = cmd
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
        
    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    
    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        
            
        
class NavigateToTower(DroneBaseBehaviour):
    def __init__(self):
        super().__init__(name = "Navigate to Tower")


class CircleAroundTower(DroneBaseBehaviour):
    def __init__(self):
        super().__init__(name = "Circle Around Tower")


class GoToPaint(DroneBaseBehaviour):
    def _init__(self):
        super().__init__(name = "Go To Paint")


class HoverAndSpray(DroneBaseBehaviour):
    def _init__(self):
        super().__init__(name = "Hover And Spray")


class GoUp(DroneBaseBehaviour):
    def __init__(self):
        super().__init__(name = "Go Up")


class ReturnHome(DroneBaseBehaviour):
    def _init__(self):
        super().__init__(name = "Return Home")
        
        
def create_tree_root():
    root = py_trees.composites.Sequence("Root")
    
    navigate_to_tower = NavigateToTower()
    circle_tower = CircleAroundTower()
    
    # Paint detection and cleanning sequence
    paint_sequence = py_trees.composites.Sequence("Paint Cleaning Sequence")
    go_to_paint = GoToPaint()
    hover_and_spray = HoverAndSpray()
    
    # Check for height, can select between going up again, or landing
    height_check = py_trees.composites.Selector("Height Check")
    go_up = GoUp()
    return_home = ReturnHome()
    
    root.add_children([navigate_to_tower, circle_tower, paint_sequence, height_check])
    paint_sequence.add_children([go_to_paint, hover_and_spray])
    height_check.add_children([go_up, return_home])
    
    return root

def main(args=None):
    print('Hi from laptop_master.')
    
    rclpy.init()
    node = rclpy.create_node('drone_behavior_tree')
    
    root = create_tree_root()
    tree = py_trees.trees.BehaviourTree(root)
    
    while rclpy.ok():
        tree.tick() # initiate tick to execute behavior tree logic
        rclpy.spin_once(node, timeout_sec=0.1) # process ROS callbacks (like messages)
        
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
