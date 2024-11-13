# servo_node.py
import rclpy
from rclpy.node import Node
from gpiozero import Servo
from time import sleep
from custom_srv_interfaces.srv import ServoControl  # Custom service

class ServoServiceNode(Node):
    def __init__(self):
        super().__init__('servo_service_node')
        
        # Declare a parameter for the servo pin with a default value
        self.declare_parameter('servo_pin', 25)
        servo_pin = self.get_parameter('servo_pin').get_parameter_value().integer_value
        self.servo = Servo(servo_pin)
        
        # Create the service
        self.srv = self.create_service(ServoControl, 'move_servo', self.move_servo_callback)
        self.get_logger().info(f"Servo control service started on pin {servo_pin}.")
        
        
    def move_servo_callback(self, request, response):
        """Move the servo to it's medium position for a requested duration"""
        
        try:
            # Move servo to mid position for a requested duration
            self.servo.mid()
            sleep(request.duration / 1000.0)
            self.get_logger().info(f"Finished moving servo for {request.duration} ms.")
            
            # Return servo to its original minum position
            self.servo.min()
            response.success = True    

        except Exception as e:
            response.success = False
        
        return response
    


def main(args = None):
    print('Hi servo service node.')
    
    # Initialize node
    rclpy.init(args = args)
    node = ServoServiceNode()   
    
    try: 
        rclpy.spin(node) # keep node running
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
