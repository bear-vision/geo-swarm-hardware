# servo_node.py
import rclpy
from rclpy.node import Node
from gpiozero import AngularServo
from time import sleep
from std_srvs.srv import Trigger
from custom_srv_interfaces.srv import ServoDurationControl, ServoAngleControl  # Custom service



class ServoServiceNode(Node):
    def __init__(self):
        super().__init__('servo_service_node')
        self.get_logger().info(f"Initialize servo node.")
        
        # Declare a parameter for the servo pin with a default value
        RPI_SERVO_PIN = 18
        
        self.SPRAYER_OFF_ANGLE = 0
        self.SPRAYER_ON_ANGLE = -10
        self.SPRAYER_MIN_ANGLE = -45
        self.SPRAYER_MAX_ANGLE = 45

        self.servo = AngularServo(RPI_SERVO_PIN, min_angle=self.SPRAYER_MIN_ANGLE, max_angle=self.SPRAYER_MAX_ANGLE)
        
        # Create the actuation with duration service
        self.servo_duration_srv = self.create_service(ServoDurationControl, 'servo_duration', self.actuate_servo_duration_callback)
        self.get_logger().info(f"Servo duration service started on pin {RPI_SERVO_PIN}.")

        # Create the sprayer on service
        self.sprayer_off_srv = self.create_service(Trigger, 'sprayer_on', self.sprayer_on_callback)
        self.get_logger().info(f"Sprayer on service started on pin {RPI_SERVO_PIN}.")

        # Create the sprayer off service
        self.sprayer_on_srv = self.create_service(Trigger, 'sprayer_off', self.sprayer_off_callback)
        self.get_logger().info(f"Sprayer off service started on pin {RPI_SERVO_PIN}.")

        self.servo_angle_srv = self.create_service(ServoAngleControl, 'servo_angle', self.actuate_servo_angle_callback)
        self.get_logger().info(f"Servo angle service started on pin {RPI_SERVO_PIN}.")

        # Initial - no airspray actuation
        self.servo.angle = self.SPRAYER_OFF_ANGLE

    def actuate_servo_duration_callback(self, request, response):
        """Move servo to sprayer on angle (actuate airspray) for a requested duration"""
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
        """Move servo to sprayer on angle to actuate sprayer"""
        try:
            self.get_logger().info(f"Actuating...")
            self.servo.angle = self.SPRAYER_ON_ANGLE
            response.success = True    
        except Exception as e:
            response.success = False
        return response

    def sprayer_off_callback(self, request, response):
        """Move servo to sprayer off angle to actuate sprayer"""
        try:
            self.get_logger().info(f"Going back to initial position...")
            self.servo.angle = self.SPRAYER_OFF_ANGLE
            response.success = True    
        except Exception as e:
            response.success = False
        return response

    
    def actuate_servo_angle_callback(self, request, response):
        """Move servo to input degree angle to actuate sprayer"""
        try:
            theta = request.theta
            if theta < self.SPRAYER_MIN_ANGLE or theta > self.SPRAYER_MAX_ANGLE:
                self.get_logger().info(f"The servo angle {request.degrees} is outside of the legal range of this servo: {(self.SPRAYER_MIN_ANGLE, self.SPRAYER_MAX_ANGLE)}.")
                response.success = False
            else:
                self.servo.angle = theta
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
