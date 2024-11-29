import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

# Lots of code in here is re-used from https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py        


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
        self.sprayer_off_srv = self.create_service(Trigger, '/rpi_master/rpi_sprayer_on', self.rpi_sprayer_on_callback)
        self.sprayer_on_srv = self.create_service(Trigger, '/rpi_master/rpi_sprayer_off', self.rpi_sprayer_off_callback)

        # Set up waypoint navigation service
        
        # Set up subscribers for Pixhawk
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Set up publishers for Pixhawk
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Class fields - some of these may not be used later
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.max_height = -1.0 #in PX4 coords for now, can change later

        # TODO - push offboard command signals when ready
        # self.timer = self.create_timer(0.1, self.offboard_command_timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Update vehicle local position."""
        self.vehicle_local_position = vehicle_local_position

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
        
        if not self.sprayer_off_client.service_is_ready():
            if not self.sprayer_off_client.wait_for_service(timeout_sec=2.0):
                response.success = False
                response.message = "Servo node sprayer_on service timed out."
        
        request = Trigger.Request()
        future = self.sprayer_on_client.call_async(request)

        future.add_done_callback(lambda f: handle_auxiliary_response(f, response))
        return response



    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    # Publishes an offboard ctrl mode message to the pixhawk.
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

        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_publisher.publish(msg)

def main(args = None):
    # Initialize node
    rclpy.init(args = args)
    node = RPiMasterNode()   
    
    try: 
        rclpy.spin(node) # keep node running
    except KeyboardInterrupt:
        node.get_logger().info("RPi Master Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
