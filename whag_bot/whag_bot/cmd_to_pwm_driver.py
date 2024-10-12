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
        self.front_left_a = 6
        self.front_left_b = 13
        self.front_left_en = 26

        self.back_left_a = 19
        self.back_left_b = 16
        self.back_left_en = 20

        self.front_right_a = 21
        self.front_right_b = 22
        self.front_right_en = 27

        self.back_right_a = 17
        self.back_right_b = 4
        self.back_right_en = 5

        # Setup GPIO mode and initialize motor pins
        GPIO.setmode(GPIO.BCM)
        motor_pins = [self.front_left_a, self.front_left_b, self.front_left_en, 
                      self.back_left_a, self.back_left_b, self.back_left_en, 
                      self.front_right_a, self.front_right_b, self.front_right_en, 
                      self.back_right_a, self.back_right_b, self.back_right_en]
        
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)

        # Initialize PWM for motor speed control (frequency: 1000 Hz)
        self.pwm_fl = GPIO.PWM(self.front_left_en, 1000)
        self.pwm_bl = GPIO.PWM(self.back_left_en, 1000)
        self.pwm_fr = GPIO.PWM(self.front_right_en, 1000)
        self.pwm_br = GPIO.PWM(self.back_right_en, 1000)

        # Start PWM with a duty cycle of 0 (stop)
        self.pwm_fl.start(0)
        self.pwm_bl.start(0)
        self.pwm_fr.start(0)
        self.pwm_br.start(0)

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
        GPIO.output(self.front_left_a, front_left_vel > 0)
        GPIO.output(self.front_left_b, front_left_vel < 0)
        self.pwm_fl.ChangeDutyCycle(min(abs(front_left_vel * 100), 100))

        # Control back left motor
        GPIO.output(self.back_left_a, back_left_vel > 0)
        GPIO.output(self.back_left_b, back_left_vel < 0)
        self.pwm_bl.ChangeDutyCycle(min(abs(back_left_vel * 100), 100))

        # Control front right motor
        GPIO.output(self.front_right_a, front_right_vel > 0)
        GPIO.output(self.front_right_b, front_right_vel < 0)
        self.pwm_fr.ChangeDutyCycle(min(abs(front_right_vel * 100), 100))

        # Control back right motor
        GPIO.output(self.back_right_a, back_right_vel > 0)
        GPIO.output(self.back_right_b, back_right_vel < 0)
        self.pwm_br.ChangeDutyCycle(min(abs(back_right_vel * 100), 100))

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
