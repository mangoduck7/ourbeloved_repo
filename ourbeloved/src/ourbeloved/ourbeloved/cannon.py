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

# index from controller_state array
# [leftX, rightX, dpadX, leftY, rightY, dpadY, 
# homeButtonState, jointButtonState, cartButtonState]

# Variables
JOINT_SPEED = 2.0  # degrees per iteration
CART_SPEED = 5.0   # mm per iteration

# controller_state[idx]
LEFT_X = 0
RIGHT_X = 1
DPAD_X = 2
LEFT_Y = 3
RIGHT_Y = 4
DPAD_Y = 5
HOME_BUTTON = 6
JOINT_BUTTON = 7
CART_BUTTON = 8

STUCK_THRESHOLD = 0.1       # degrees
STUCK_COUNTER_MAX = 100  
JOINT_THRESHOLD = 1.0       # degrees
ITERATIONS = 100
GOAL_THRESHOLD = 19         # mm

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


    # --- Helper Functions ----
    def check_goal(self, goalX, goalY, goalZ):
        goal_joints = 0

        
        return goal_joints



    def controller_state_callback(self, msg):
        # if we're meant to be in precise mode, exit out of function
        if self.precise_mode == True:
            return None
        
        data = msg.data

        # what mode are we in
        if data[JOINT_BUTTON] == 1 and self.prev_joint_button == 0:
            self.mode = 'joint'
            self.get_logger().info('Switched to joint mode')  
        if data[CART_BUTTON] == 1 and self.prev_cart_button == 0:
            self.mode = 'cartesian'
            self.get_logger().info('Switched to cartestian mode')
        
        # Update previous button states 
        self.prev_joint_button = data[JOINT_BUTTON]
        self.prev_cart_button = data[CART_BUTTON]

        # Detect button press for homing
        if data[HOME_BUTTON] == 1 and self.prev_home_button == 0:
            self.xarm.home()
            self.get_logger().info('Homing...')
        self.prev_home_button = data[HOME_BUTTON]  # update state



    def precise_joint_callback(self, msg):
        self.precise_mode = True


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Cannon()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()