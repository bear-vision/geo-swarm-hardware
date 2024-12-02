import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from custom_interfaces.action import DroneNavigateToWaypoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude
from enum import Enum
import numpy as np
from rpi_master import rpi_master_utils

# for use when implementing multithreading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

# Lots of code in here is re-used from https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py     

# TODO:
# (IMPORTANT) implement MultiThreadedExecutor and use threading.lock to handle issues with resource sharing and concurrency
# Consider adding another action just for landing safely


class DroneFlightState(Enum):
    GROUNDED = 0 #idle, on the ground
    NAVIGATING = 1 #currently trying to fly towards a waypoint
    HOVERING = 2 #maintaining current position

class RPiMasterNode(Node):

    def __init__(self):
        super().__init__('rpi_master_node')
        self.get_logger().info(f"Initialize RPi Master Node.")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Set up service proxies for servo node
        self.sprayer_on_client = self.create_client(Trigger, 'sprayer_on')
        self.sprayer_off_client = self.create_client(Trigger, 'sprayer_off')
        if not self.sprayer_on_client.wait_for_service(timeout_sec = 2.0):
            self.get_logger().warn("Sprayer on client not set up.")
        else:
            self.get_logger().info("Sprayer on client set up.")
        if not self.sprayer_off_client.wait_for_service(timeout_sec = 2.0):
            self.get_logger().warn("Sprayer off client not set up.")
        else:
            self.get_logger().info("Sprayer off client set up.")

        # Set up sprayer services that rpi can control
        self.sprayer_off_srv = self.create_service(Trigger, '/rpi_master/rpi_sprayer_off', self.rpi_sprayer_off_callback)
        self.sprayer_on_srv = self.create_service(Trigger, '/rpi_master/rpi_sprayer_on', self.rpi_sprayer_on_callback)

        # Class fields 
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_status = VehicleStatus()
        self.flight_state = DroneFlightState.GROUNDED

        self.current_goal = None # Example implementations use goal queues. We only need 1 goal at a time
        
        #geometry_msgs/Pose objs
        self.latest_waypoint = None
        self.prev_waypoint = None

        self.flight_state_lock = threading.Lock()

        # Constants
        self.MAX_HEIGHT = -1.0 # max altitude allowed, can adjust this as necessary
        self.POSITION_CONTROL_MAX_POSITION_ERROR = 0.3 # max allowable error in meters (euclidean dist). TODO: tune
        self.POSITION_CONTROL_MAX_ORIENTATION_ERROR = 0.2 # max allowable error in radians (manhattan dist). TODO: tune

        # Set up subscribers for Pixhawk
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Set up publishers for Pixhawk
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Set up timers to publish relevant signals
        self.offboard_command_timer = self.create_timer(0.1, self.offboard_command_timer_callback)
        self.latest_waypoint_timer = self.create_timer(0.1, self.latest_waypoint_timer_callback)

        # Set up waypoint navigation action server
        self.waypoint_action_server = ActionServer(self, 
            DroneNavigateToWaypoint, 
            'drone_navigate_to_waypoint',
            handle_accepted_callback = self.handle_accepted_navigate_callback,
            execute_callback = self.execute_navigate_callback,
            goal_callback = self.navigate_goal_callback,
            cancel_callback = self.cancel_navigate_callback,
            callback_group = MutuallyExclusiveCallbackGroup() #only allow serial execution of server callbacks
        )

    def destroy(self):
        self.waypoint_action_server.destroy()
        super().destroy_node()

    def navigate_goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        with self.flight_state_lock:
            if self.flight_state is not DroneFlightState.NAVIGATING and self.current_goal is None:
                # TODO - perform some validation on the goal request (for example, we should never allow requesting below or above a certain height)
                self.current_goal = rpi_master_utils.px4_to_ros_transform(goal_request)

                self.flight_state = DroneFlightState.NAVIGATING
                return GoalResponse.ACCEPT
            else:
                # If we are currently navigating, do not allow another request
                return GoalResponse.REJECT 

    def handle_accepted_navigate_callback(self, goal_handle):
        '''Start or defer execution of an already-accepted goal'''

        # update waypoint fields. self.publish_latest_waypoint() and related timer callback handles the actual publishing of the correct waypoint to PX4.
        self.prev_waypoint = self.latest_waypoint
        # convert waypoint to px4 coordinates 
        self.latest_waypoint = rpi_master_utils.ros_to_px4_world_frame_transform(goal_handle.request.waypoint)

        # go to execute callback
        self.current_goal.execute()

    def cancel_navigate_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        return CancelResponse.ACCEPT
    
    def execute_navigate_callback(self, goal_handle):
        """Executes the navigation callback. Monitors the current position of the drone vs final desired position"""

        def pose_distance(pose_a: Pose, pose_b: Pose):
            # returns the two metrics of pose distance that we care about:
            # 1. distance in position (euclidean xyz) - in meters
            # 2. distance in yaw (manhattan yaw) - in radians
            position_error = np.sqrt((pose_a.position.x - pose_b.position.x) ** 2 + (pose_a.position.y - pose_b.position.y) ** 2 + (pose_a.position.z - pose_b.position.z) ** 2)
            a_yaw, b_yaw = rpi_master_utils.euler_from_quaternion(pose_a.orientation)[2], rpi_master_utils.euler_from_quaternion(pose_b.orientation)[2]
            orientation_error = np.abs(a_yaw - b_yaw)

            return position_error, orientation_error

        try:
            self.get_logger().info("Executing goal...")

            #create a rate object for checking error range and goal cancellation
            error_check_rate = self.create_rate(20, self.get_clock())

            # curr_pose and latest_waypoint are both in px4 coordinates
            curr_pose = self.get_current_pose()
            position_error, orientation_error = pose_distance(self.latest_waypoint, curr_pose)

            while position_error > self.POSITION_CONTROL_MAX_POSITION_ERROR or orientation_error > self.POSITION_CONTROL_MAX_ORIENTATION_ERROR:

                # check if cancel request has come through
                if goal_handle.is_cancel_requested:
                    with self.flight_state_lock:
                        goal_handle.canceled()  
                        self.get_logger().info("Goal was canceled.")

                        #restore prev waypoint to allow for hovering there? TODO define correct behavior
                        self.latest_waypoint = self.prev_waypoint
                        self.flight_state = DroneFlightState.HOVERING
                        self.current_goal = None

                    result = DroneNavigateToWaypoint.Result()
                    result.success = False
                    result.message = "Navigate to waypoint action was canceled."
                    return result

                # populate and publish feedback
                feedback_msg = DroneNavigateToWaypoint.Feedback()
                # transform current pose from px4 to ros2
                feedback_msg.current_pose = rpi_master_utils.px4_to_ros_world_frame_transform(curr_pose)
                goal_handle.publish_feedback(feedback_msg)

                # update curr_pose and errors
                curr_pose = self.get_current_pose()
                position_error, orientation_error = pose_distance(self.latest_waypoint, curr_pose)

                # sleep for a tiny bit 
                error_check_rate.sleep()

            #TODO - define behavior for being at the waypoint - probably want to be within the error range for some amt of time.
            # current implementation just returns success upon first time getting within the error range.

            goal_handle.succeed()

            # Populate result message
            result = DroneNavigateToWaypoint.Result()
            result.success = True
            result.message = "Succesfully navigated to waypoint."

            with self.flight_state_lock:
                self.flight_state = DroneFlightState.HOVERING

            return result
        except Exception as e:
            # handle errors/exceptions - TODO define desired behavior.
            self.get_logger().error(f"Error encountered in executing navigation callback: {e}")
        finally:
            # unset current goal/waypoint.
            self.current_goal = None

        
    def get_current_pose(self):
        """Get current pose of the drone as a geometry_msgs/msg/Pose object. Returns the current pose in PX4 frame."""
        # TODO use threading to handle concurrency correctly
        pose_obj = Pose()
        pose_obj.position.x = self.vehicle_local_position.x
        pose_obj.position.y = self.vehicle_local_position.y
        pose_obj.position.z = self.vehicle_local_position.z

        pose_obj.orientation.w = self.vehicle_attitude.q[0]
        pose_obj.orientation.x = self.vehicle_attitude.q[1]
        pose_obj.orientation.y = self.vehicle_attitude.q[2]
        pose_obj.orientation.z = self.vehicle_attitude.q[3]

        return pose_obj

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Update vehicle local position."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Update vehicle attitude (orientation)."""
        self.vehicle_attitude = vehicle_attitude

    def vehicle_status_callback(self, vehicle_status):
        """Update vehicle status."""
        self.vehicle_status = vehicle_status

    def publish_vehicle_command(self, command_id, *args):
        '''
            Publishes a vehicle command to the Pixhawk.
        '''
        msg = VehicleCommand()

        msg.command = command_id
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        # Populate msg params - see https://github.com/PX4/px4_msgs/blob/release/1.14/msg/VehicleCommand.msg#L165C1-L171C79
        # Only need the first 7 args. if needed, pad args until length 7. default value for float types is 0 anyways.
        args = (args + (0.,) * max(0, 7 - len(args)))[:7]
        self.get_logger().info(f"args: {args}")
        msg.param1, msg.param2, msg.param3, msg.param4, msg.param5, msg.param6, msg.param7 = args

        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        '''
            Sends an arm command to the drone. Note that arming only happens when all pre-flight checks pass, so this isn't guaranteed to work.
        '''

        # https://github.com/PX4/px4_msgs/blob/ffb6e80e1c17e5714395611a020c282a87af8fa4/msg/VehicleCommand.msg#L78C1-L78C99
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent.")

    def disarm(self):
        '''
            Disarms the drone.
        '''

        # https://github.com/PX4/px4_msgs/blob/ffb6e80e1c17e5714395611a020c282a87af8fa4/msg/VehicleCommand.msg#L78C1-L78C99
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent.")

    def rpi_sprayer_on_callback(self, request, response):
        def handle_auxiliary_response(future, response):
            try:
                auxiliary_response = future.result()
                if auxiliary_response.success:  # Assuming the auxiliary service has a boolean `success` field
                    # self.get_logger().info("Sprayer on call success.")
                    response.message = "Success"
                    response.success = True
                else:
                    self.get_logger().warn("Sprayer on call failed.")
                    response.message = "Sprayer on call failed."
                    response.success = False 
            except Exception as e:
                response.message = "Could not call sprayer_on service"
                self.get_logger().error(f"Failed to call servo sprayer on service: {e}")
                response.success = False
        
        if not self.sprayer_on_client.service_is_ready():
            if not self.sprayer_on_client.wait_for_service(timeout_sec=2.0):
                response.success = False
                response.message = "Servo node sprayer_on service timed out."
        
        #never allow spraying unless we're hovering
        if not self.flight_state == DroneFlightState.HOVERING:
            response.success = False
            response.message = "Cannot use sprayer outside of hovering flight state."

        request = Trigger.Request()
        future = self.sprayer_on_client.call_async(request)

        future.add_done_callback(lambda f: handle_auxiliary_response(f, response))
        return response

    def rpi_sprayer_off_callback(self, request, response):
        def handle_auxiliary_response(future, response):
            try:
                auxiliary_response = future.result()
                if auxiliary_response.success:
                    # self.get_logger().info("Sprayer off call success.")
                    response.message = "Success"
                    response.success = True  # Set the response based on auxiliary service success
                else:
                    self.get_logger().warn("Sprayer off call failed.")
                    response.message = "Sprayer off call failed."
                    response.success = False 
            except Exception as e:
                response.message = "Could not call sprayer_off service"
                self.get_logger().error(f"Failed to call servo sprayer off service: {e}")
                response.success = False
            
            return response 
        
        if not self.sprayer_off_client.service_is_ready():
            if not self.sprayer_off_client.wait_for_service(timeout_sec=2.0):
                response.success = False
                response.message = "Servo node sprayer_on service timed out."
                return response 
        
        request = Trigger.Request()
        future = self.sprayer_on_client.call_async(request)

        future.add_done_callback(lambda f: handle_auxiliary_response(f, response))
        return response

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_mode(self):
        '''
            PX4 needs to receive an offboard control message every second or so to enable offboard control on the flight controller.

            This is the Python version of C++ example code in
               https://docs.px4.io/main/en/ros2/offboard_control.html
        '''
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.actuator = False

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_latest_waypoint(self):
        '''
            Publishes the latest waypoint (stored in the class field self.latest_waypoint)
        '''

        if not self.latest_waypoint:
            self.get_logger().warn("Tried to call publish_latest_waypoint without a valid latest waypoint. This call will do nothing.")
            return

        waypoint_pose = self.latest_waypoint
        msg = TrajectorySetpoint()
        msg.x, msg.y, msg.z = waypoint_pose.position.x, waypoint_pose.position.y, waypoint_pose.position.z
        msg.yaw = rpi_master_utils.euler_from_quaternion(waypoint_pose.orientation)[2]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def offboard_command_timer_callback(self):
        #publish offboard mode commands if we are not in a GROUNDED state.
        if self.flight_state is not DroneFlightState.GROUNDED:
            self.publish_offboard_control_mode()

    def latest_waypoint_timer_callback(self):
        current_flight_state = None
        with self.flight_state_lock:
            current_flight_state = self.flight_state
        if current_flight_state is not DroneFlightState.GROUNDED and self.latest_waypoint is not None:
            self.publish_latest_waypoint()

def main(args = None):
    # Initialize node
    rclpy.init(args = args)
    node = RPiMasterNode()   

    # Create a MultiThreadedExecutor to handle both nodes concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try: 
        executor.spin() # keep node running
    except KeyboardInterrupt:
        node.get_logger().info("RPi Master Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
