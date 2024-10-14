import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class OmniTeleop(Node):
    def __init__(self):
        super().__init__('omni_teleop')
        
        # Publisher to the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initializing terminal settings for keyboard input
        self.settings = termios.tcgetattr(sys.stdin)

        # Speed parameters
        self.speed = 0.5  # Linear speed
        self.turn = 1.0   # Angular speed
        
        self.msg = """
        Control Your Omnidirectional Robot
        ----------------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .
        
        i : move forward
        , : move backward
        j : strafe left
        l : strafe right
        u/o : rotate counter-clockwise/clockwise
        k : stop

        CTRL-C to quit
        """
        
        # Movement bindings for omnidirectional robot
        self.move_bindings = {
            'i': (1, 0, 0, 0),    # Move forward
            ',': (-1, 0, 0, 0),   # Move backward
            'j': (0, 1, 0, 0),    # Strafe left
            'l': (0, -1, 0, 0),   # Strafe right
            'u': (0, 0, 0, 1),    # Rotate counterclockwise
            'o': (0, 0, 0, -1),   # Rotate clockwise
            'k': (0, 0, 0, 0),    # Stop
        }

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(self.msg)
        x = 0
        y = 0
        z = 0
        th = 0

        try:
            while True:
                key = self.get_key()

                if key in self.move_bindings:
                    x, y, z, th = self.move_bindings[key]
                else:
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                    if key == '\x03':  # Handle CTRL+C
                        break

                # Create and publish Twist message
                twist = Twist()
                twist.linear.x = x * self.speed
                twist.linear.y = y * self.speed
                twist.angular.z = th * self.turn
                self.publisher_.publish(twist)

        except Exception as e:
            print(e)

        finally:
            # Reset to stop the robot when exiting
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.angular.z = 0
            self.publisher_.publish(twist)
            
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)

    omni_teleop = OmniTeleop()
    omni_teleop.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
