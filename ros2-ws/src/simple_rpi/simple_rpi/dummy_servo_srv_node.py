# creates turn on and turn off sprayer dummy service

import rclpy
from rclpy.node import Node
from time import sleep
from std_srvs.srv import Trigger
from custom_interfaces.srv import ServoDurationControl, ServoAngleControl  # Custom service


class DummyServoServiceNode(Node):
    def __init__(self):
        super().__init__('dummy_servo_service_node')
        self.get_logger().info(f"Initialize dummy servo node.")

        # Create the sprayer on service
        self.sprayer_off_srv = self.create_service(Trigger, 'sprayer_on', self.sprayer_on_callback)
        self.get_logger().info(f"Sprayer on service started")

        # Create the sprayer off service
        self.sprayer_on_srv = self.create_service(Trigger, 'sprayer_off', self.sprayer_off_callback)
        self.get_logger().info(f"Sprayer off service started")

        self.get_logger().info(f"Servo Node up")
        


    def sprayer_on_callback(self, request, response):
        """Move servo to sprayer on angle to actuate sprayer"""
        try:
            self.get_logger().info(f"Actuating sprayer...")
            response.success = True    
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Error turning sprayer on: {str(e)}")
        return response

    def sprayer_off_callback(self, request, response):
        """Move servo to sprayer off angle to actuate sprayer"""
        try:
            self.get_logger().info(f"Turn off sprayer.")
            response.success = True    
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Error turning sprayer off: {str(e)}")
        return response

    

def main(args = None):
    # Initialize node
    rclpy.init(args = args)
    node = DummyServoServiceNode()   
    
    try: 
        rclpy.spin(node) # keep node running
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
