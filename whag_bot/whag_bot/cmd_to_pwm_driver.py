import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist

class VelocitySubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        
        # Subscription to /cmd_vel topic
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_to_pwm_callback, 10)

        # Define GPIO pin assignments for the omnidirectional robot's motors
        self.front_left_in3 = 17  # Front-left motor IN3
        self.front_left_in4 = 4   # Front-left motor IN4

        self.back_left_in1 = 22   # Back-left motor IN1
        self.back_left_in2 = 27   # Back-left motor IN2

        self.front_right_in1 = 26 # Front-right motor IN1
        self.front_right_in2 = 19 # Front-right motor IN2

        self.back_right_in3 = 5   # Back-right motor IN3
        self.back_right_in4 = 6   # Back-right motor IN4

        self.en_pin = 13  # Single PWM enable pin for all motors

        # Setup GPIO mode and initialize motor pins
        GPIO.setmode(GPIO.BCM)
        motor_pins = [self.front_left_in3, self.front_left_in4, 
                      self.back_left_in1, self.back_left_in2, 
                      self.front_right_in1, self.front_right_in2, 
                      self.back_right_in3, self.back_right_in4, self.en_pin]
        
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)  # Set all motors to LOW initially

        # Initialize PWM for motor speed control (frequency: 1000 Hz)
        self.pwm = GPIO.PWM(self.en_pin, 1000)

        # Start PWM with a duty cycle of 0 (stop)
        self.pwm.start(0)

    def cmd_to_pwm_callback(self, msg):
        # Extract linear and angular velocities from Twist message
        linear_x = msg.linear.x   # Forward/backward
        linear_y = msg.linear.y   # Left/right strafing
        angular_z = msg.angular.z # Rotational velocity

        # Calculate individual wheel velocities for omnidirectional movement
        front_left_vel = linear_x - linear_y - angular_z
        back_left_vel = linear_x + linear_y - angular_z
        front_right_vel = linear_x + linear_y + angular_z
        back_right_vel = linear_x - linear_y + angular_z

        # Control front left motor
        GPIO.output(self.front_left_in4, front_left_vel > 0)
        GPIO.output(self.front_left_in3, front_left_vel < 0)

        # Control back left motor
        GPIO.output(self.back_left_in1, back_left_vel > 0)
        GPIO.output(self.back_left_in2, back_left_vel < 0)

        # Control front right motor
        GPIO.output(self.front_right_in2, front_right_vel > 0)
        GPIO.output(self.front_right_in1, front_right_vel < 0)

        # Control back right motor
        GPIO.output(self.back_right_in3, back_right_vel > 0)
        GPIO.output(self.back_right_in4, back_right_vel < 0)

        # Adjust motor speed based on velocity magnitude
        max_vel = max(abs(front_left_vel), abs(back_left_vel), abs(front_right_vel), abs(back_right_vel))
        self.pwm.ChangeDutyCycle(min(max_vel * 100, 100))  # Scale PWM duty cycle

def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = VelocitySubscriber()

    try:
        rclpy.spin(velocity_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_subscriber.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()  # Clean up GPIO when the program exits

if __name__ == '__main__':
    main()
