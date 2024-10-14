import RPi.GPIO as GPIO
import time

# Pin definitions for each motor
front_left_in3 = 17
front_left_in4 = 4
back_left_in1 = 22
back_left_in2 = 27
front_right_in1 = 26
front_right_in2 = 19
back_right_in3 = 5
back_right_in4 = 6
en_pin = 13  # Single PWM enable pin for all motors

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set up motor pins as output
motor_pins = [front_left_in3, front_left_in4, back_left_in1, back_left_in2, 
              front_right_in1, front_right_in2, back_right_in3, back_right_in4, en_pin]

for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# Initialize PWM for motor speed control on the enable pin
pwm = GPIO.PWM(en_pin, 1000)  # Set PWM frequency to 1000 Hz
pwm.start(75)  # Start PWM with 75% duty cycle for all motors

# Movement functions
def forward(seconds):
    print("Moving Forward")
    GPIO.output(front_left_in4, GPIO.HIGH)
    GPIO.output(front_left_in3, GPIO.LOW)
    GPIO.output(back_left_in1, GPIO.HIGH)
    GPIO.output(back_left_in2, GPIO.LOW)
    
    GPIO.output(front_right_in2, GPIO.HIGH)
    GPIO.output(front_right_in1, GPIO.LOW)
    GPIO.output(back_right_in3, GPIO.HIGH)
    GPIO.output(back_right_in4, GPIO.LOW)
    
    time.sleep(seconds)

def backward(seconds):
    print("Moving Backward")
    GPIO.output(front_left_in4, GPIO.LOW)
    GPIO.output(front_left_in3, GPIO.HIGH)
    GPIO.output(back_left_in1, GPIO.LOW)
    GPIO.output(back_left_in2, GPIO.HIGH)
    
    GPIO.output(front_right_in2, GPIO.LOW)
    GPIO.output(front_right_in1, GPIO.HIGH)
    GPIO.output(back_right_in3, GPIO.LOW)
    GPIO.output(back_right_in4, GPIO.HIGH)
    
    time.sleep(seconds)

def strafe_right(seconds):
    print("Strafing Right")
    GPIO.output(front_left_in3, GPIO.LOW)
    GPIO.output(front_left_in4, GPIO.HIGH)
    GPIO.output(back_right_in4, GPIO.LOW)
    GPIO.output(back_right_in3, GPIO.HIGH)

    GPIO.output(front_right_in1, GPIO.HIGH)
    GPIO.output(front_right_in2, GPIO.LOW)
    GPIO.output(back_left_in2, GPIO.HIGH)
    GPIO.output(back_left_in1, GPIO.LOW)
    
    time.sleep(seconds)

def strafe_left(seconds):
    print("Strafing Left")
    GPIO.output(front_left_in3, GPIO.HIGH)
    GPIO.output(front_left_in4, GPIO.LOW)
    GPIO.output(back_right_in4, GPIO.HIGH)
    GPIO.output(back_right_in3, GPIO.LOW)

    GPIO.output(front_right_in1, GPIO.LOW)
    GPIO.output(front_right_in2, GPIO.HIGH)
    GPIO.output(back_left_in2, GPIO.LOW)
    GPIO.output(back_left_in1, GPIO.HIGH)
    
    time.sleep(seconds)

def stop_all():
    print("Stopping Motors")
    pwm.ChangeDutyCycle(0)

def cleanup():
    print("Cleaning up GPIO")
    pwm.stop()
    GPIO.cleanup()

# Main function to test movements
def main():
    forward(2)
    backward(2)
    strafe_left(2) 
    strafe_right(2)
    stop_all()
    
    
    cleanup()

if __name__ == '__main__':
    main()
