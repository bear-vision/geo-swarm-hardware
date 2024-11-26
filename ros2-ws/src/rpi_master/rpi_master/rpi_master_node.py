import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
#TODO: import some custom message or service type for servo control


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

        #TODO: Set up Service Proxy for servo node
        self.sprayer_on_client = self.create_client(Trigger, 'sprayer_on')
        self.sprayer_off_client = self.create_client(Trigger, 'sprayer_off')
        

        #TODO: Set up Subscribers (or service!) for RPi to listen to laptop
        # self.create_subscription(Empty, 'temporary_rpi_topic', self.laptop_command_callback, 10)

        # Set up Subscribers for Pixhawk
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        #TODO: Set up Publishers for Pixhawk
        # following the example in https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp for now
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)


        # Create a timer to push offboard command mode messages if self.pi_command_mode is true every 0.5 seconds
        self.timer = self.create_timer(0.1, self.offboard_command_timer_callback)

        # Other class fields
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # temporary fields for hardware testing
        self.reached_hover_pt = False
        self.hover_pt_reach_start_time = -1
        self.takeoff_height = -1.0 #1 meter above where the drone starts


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
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
            Arms the drone.

            Python implementation of C++ Example at https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp

        '''

        # https://github.com/PX4/px4_msgs/blob/ffb6e80e1c17e5714395611a020c282a87af8fa4/msg/VehicleCommand.msg#L78C1-L78C99
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent.")
    

    def disarm(self):
        '''
            Disarms the drone.

            Python implementation of C++ Example at https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp
        '''

        # https://github.com/PX4/px4_msgs/blob/ffb6e80e1c17e5714395611a020c282a87af8fa4/msg/VehicleCommand.msg#L78C1-L78C99
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Arm command sent.")

    def sprayer_on(self):
        while not self.sprayer_on_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sprayer on service not available, waiting again...')
        request = Trigger.Request()

        future = self.sprayer_on_client.call_async(request)

        # add a callback upon completion - not sure if necessary
        # future.add_done_callback(self.callback)

    def sprayer_off(self):
        while not self.sprayer_off_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sprayer off service not available, waiting again...')
        request = Trigger.Request()

        future = self.sprayer_off_client.call_async(request)

        # add a callback upon completion - not sure if necessary
        # future.add_done_callback(self.callback)

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    # TODO: fill out when we have a better idea of how to implement 
    def laptop_command_callback(self, message):
        pass


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

    def publish_hover_message(self):
        '''
            Publishes a trajectory setpoint to the pixhawk, telling it to hover at 1 meter above the ground if possible.
        
            This is the Python version of C++ example code in
                https://docs.px4.io/main/en/ros2/offboard_control.html
        '''        

        msg = TrajectorySetpoint()

        #makes the drone hover at 1 meter in world coordinates (note: PX4 coordinate frame)
        msg.position = [0.0, 0.0, -1.0]
        msg.yaw = -3.14

        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        self.trajectory_setpoint_publisher.publish(msg)


    def offboard_command_timer_callback(self):
        # if self.offboard_control:
        #     self.publish_offboard_control_mode()
        #     self.get_logger().debug('RPi Master published offboard command mode to Pixhawk.')
        
        if self.offboard_setpoint_counter == 10:
            self.get_logger().info("RPi Master timer callback 10th call!")

            #send the command to switch to offboard mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

            #arm
            self.arm()
        

        self.publish_offboard_control_mode()

        #either try to get to hover pt, or track how long we have 
        if not self.reached_hover_pt:
            self.publish_hover_message()

            if self.vehicle_local_position.z <= -1.0:
                self.reached_hover_pt = True 

                #sets the start time in milliseconds
                self.hover_pt_reach_start_time = self.get_clock().now().nanoseconds // 1_000_000
        else:
            if (self.get_clock().now().nanoseconds // 1_000_000) - self.hover_pt_reach_start_time > 3000:
                # we have been hovering for 3 seconds - time to land
                self.land()
                exit(0) # TODO: verify if this is the behavior we want in the test            

       
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    



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
