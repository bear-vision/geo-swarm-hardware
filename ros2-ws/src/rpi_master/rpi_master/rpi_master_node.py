import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Empty
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
#TODO: import some custom message or service type for servo control


class RPiMasterNode(Node):

    def __init__(self):
        super().__init__('rpi_master_node')
        self.get_logger().info(f"Initialize RPi Master Node.")

        #TODO: Set up Service Proxy for servo node
        self.sprayer_on_client = self.create_client(Trigger, 'sprayer_on')
        self.sprayer_off_client = self.create_client(Trigger, 'sprayer_off')
        

        #TODO: Set up Subscribers (or service!) for RPi to listen to laptop
        self.create_subscription(Empty, 'temporary_rpi_topic', self.laptop_command_callback, 10)

        #TODO: Set up Publishers for Pixhawk
        # following the example in https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp for now
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', 10)

        #A boolean flag to control whether or not we should tell the Pi to be in offboard mode. Not sure if this is useful yet.
        self.offboard_control = True

        # Create a timer to push offboard command mode messages if self.pi_command_mode is true every 0.5 seconds
        self.timer = self.create_timer(0.5, self.offboard_command_timer_callback)
    
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
        msg.position = [0, 0, -1]
        msg.yaw = -3.14

        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        self.trajectory_setpoint_publisher.publish(msg)

    def offboard_command_timer_callback(self):
        if self.offboard_control:
            self.publish_offboard_control_mode()
            self.get_logger().debug('RPi Master published offboard command mode to Pixhawk.')
    



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
