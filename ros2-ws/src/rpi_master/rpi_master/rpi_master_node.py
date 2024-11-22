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
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', 10)

    
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
