import RPi.GPIO as GPIO
import time

# Pin definitions for each motor
front_left_a = 6
front_left_b = 13
front_left_en = 26

back_left_a = 19
back_left_b = 16
back_left_en = 20

front_right_a = 21
front_right_b = 22
front_right_en = 27

back_right_a = 17
back_right_b = 4
back_right_en = 5

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Set up motor pins as output
motor_pins = [front_left_a, front_left_b, front_left_en, back_left_a, back_left_b, back_left_en, 
              front_right_a, front_right_b, front_right_en, back_right_a, back_right_b, back_right_en]

for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# Initialize PWM for motor speed control
pwm_fl = GPIO.PWM(front_left_en, 1000)
pwm_bl = GPIO.PWM(back_left_en, 1000)
pwm_fr = GPIO.PWM(front_right_en, 1000)
pwm_br = GPIO.PWM(back_right_en, 1000)

# Start PWM with 75% duty cycle
pwm_fl.start(75)
pwm_bl.start(75)
pwm_fr.start(75)
pwm_br.start(75)

# Movement functions
def forward(second):
    print("Moving Forward")
    # Front left and back left motors forward, front right and back right motors forward
    GPIO.output(front_left_a, GPIO.HIGH)
    GPIO.output(front_left_b, GPIO.LOW)
    GPIO.output(back_left_a, GPIO.HIGH)
    GPIO.output(back_left_b, GPIO.LOW)
    
    GPIO.output(front_right_a, GPIO.HIGH)
    GPIO.output(front_right_b, GPIO.LOW)
    GPIO.output(back_right_a, GPIO.HIGH)
    GPIO.output(back_right_b, GPIO.LOW)
    time.sleep(second)

def backward(second):
    print("Moving Backward")
    # Reverse all motors
    GPIO.output(front_left_a, GPIO.LOW)
    GPIO.output(front_left_b, GPIO.HIGH)
    GPIO.output(back_left_a, GPIO.LOW)
    GPIO.output(back_left_b, GPIO.HIGH)
    
    GPIO.output(front_right_a, GPIO.LOW)
    GPIO.output(front_right_b, GPIO.HIGH)
    GPIO.output(back_right_a, GPIO.LOW)
    GPIO.output(back_right_b, GPIO.HIGH)
    time.sleep(second)

def strafe_left(second):
    print("Strafing Left")
    # Motors configuration for moving left (front left and back right reverse, front right and back left forward)
    GPIO.output(front_left_a, GPIO.LOW)
    GPIO.output(front_left_b, GPIO.HIGH)
    GPIO.output(back_right_a, GPIO.LOW)
    GPIO.output(back_right_b, GPIO.HIGH)

    GPIO.output(front_right_a, GPIO.HIGH)
    GPIO.output(front_right_b, GPIO.LOW)
    GPIO.output(back_left_a, GPIO.HIGH)
    GPIO.output(back_left_b, GPIO.LOW)
    time.sleep(second)

def strafe_right(second):
    print("Strafing Right")
    # Motors configuration for moving right (front left and back right forward, front right and back left reverse)
    GPIO.output(front_left_a, GPIO.HIGH)
    GPIO.output(front_left_b, GPIO.LOW)
    GPIO.output(back_right_a, GPIO.HIGH)
    GPIO.output(back_right_b, GPIO.LOW)

    GPIO.output(front_right_a, GPIO.LOW)
    GPIO.output(front_right_b, GPIO.HIGH)
    GPIO.output(back_left_a, GPIO.LOW)
    GPIO.output(back_left_b, GPIO.HIGH)
    time.sleep(second)

def stop_all():
    print("Stopping Motors")
    pwm_fl.ChangeDutyCycle(0)
    pwm_bl.ChangeDutyCycle(0)
    pwm_fr.ChangeDutyCycle(0)
    pwm_br.ChangeDutyCycle(0)

def cleanup():
    print("Cleaning up GPIO")
    GPIO.cleanup()

# Main function to test movements
def main():
    forward(2)
    stop_all()
    time.sleep(1)
    
    backward(2)
    stop_all()
    time.sleep(1)
    
    strafe_left(2)
    stop_all()
    time.sleep(1)
    
    strafe_right(2)
    stop_all()
    time.sleep(1)
    
    cleanup()

if __name__ == '__main__':
    main()
