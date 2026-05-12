import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

from ourbeloved.wx250s_kinematics import fk, ik
from xarmclient import XArm

class Cannon(Node):

    def __init__(self):
        super().__init__("cannon")

        # Subscribers
        self.controller_state_sub = self.create_subscription(
            Float64MultiArray, 
            "/controller_state", 
            self.listener_callback, 10)
        self.precise_joint_sub = self.create_subscription(
            JointState, 
            "/precise_joint_cmd", 
            self.listener_callback, 10)
        
        # Robot
        self.xarm = XArm()

        # Modes
        self.mode = 'joint'
        self.precise_mode = False

        # Button press detection
        self.prev_home_button = 0
        self.prev_joint_button = 0
        self.prev_cart_button = 0

        # Timer (generic template)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def listener_callback(self, msg):
        self.controller_state_sub = 

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Cannon()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()