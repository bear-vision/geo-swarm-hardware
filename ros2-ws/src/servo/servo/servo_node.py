# servo_node.py
import rclpy
from rclpy.node import Node
from gpiozero import AngularServo
from time import sleep
from std_srvs.srv import Trigger
from custom_srv_interfaces.srv import ServoControl  # Custom service



class ServoServiceNode(Node):
    def __init__(self):
        super().__init__('servo_service_node')
        self.get_logger().info(f"Initialize servo node.")
        
        # Declare a parameter for the servo pin with a default value
        RPI_SERVO_PIN = 18
        
        self.SPRAYER_OFF_ANGLE = 90
        self.SPRAYER_ON_ANGLE = 80

        self.servo = AngularServo(RPI_SERVO_PIN, min_angle=-90, max_angle=90)
        
        # Create the actuation with duration service
        self.srv = self.create_service(ServoControl, 'servo_duration', self.actuate_servo_duration_callback)
        self.get_logger().info(f"Servo duration service started on pin {RPI_SERVO_PIN}.")

        # Create the sprayer on service
        self.srv = self.create_service(Trigger, 'sprayer_on', self.sprayer_on_callback)
        self.get_logger().info(f"Sprayer on service started on pin {RPI_SERVO_PIN}.")

        # Create the sprayer off service
        self.srv = self.create_service(Trigger, 'sprayer_off', self.sprayer_off_callback)
        self.get_logger().info(f"Sprayer off service started on pin {RPI_SERVO_PIN}.")

        # Initial - no airspray actuation
        self.servo.angle = 90
        
        
    def actuate_servo_duration_callback(self, request, response):
        """Move servo to 80 degrees (actuate airspray) for a requested duration"""
        try:
            self.servo.angle = self.SPRAYER_ON_ANGLE
            sleep(request.duration / 1000.0)
            self.get_logger().info(f"Finished moving servo for {request.duration} ms.")
            
            # Return servo to its original minum position
            self.servo.angle = self.SPRAYER_OFF_ANGLE
            response.success = True    

        except Exception as e:
            response.success = False
        
        return response

    def sprayer_on_callback(self, request, response):
        """Move servo to 80 degrees to actuate sprayer"""
        try:
            self.get_logger().info(f"Actuating...")
            self.servo.angle = self.SPRAYER_ON_ANGLE
            response.success = True    
        except Exception as e:
            response.success = False
        return response

    def sprayer_off_callback(self, request, response):
        """Move servo to 90 degrees to actuate sprayer"""
        try:
            self.get_logger().info(f"Going back to initial position...")
            self.servo.angle = self.SPRAYER_OFF_ANGLE
            response.success = True    
        except Exception as e:
            response.success = False
        return response

    


def main(args = None):
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
