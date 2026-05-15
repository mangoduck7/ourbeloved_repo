import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

# Variables
JOINT_SPEED = 2.0  # degrees per iteration
CART_SPEED = 5.0   # mm per iteration

# axes[idx]
LEFT_X = 0
LEFT_Y = 1
RIGHT_X = 3
RIGHT_Y = 4
DPAD_X = 6
DPAD_Y = 7

# buttons[idx]
HOME_IDX = 10  
JOINT_BUTTON = 8
CART_BUTTON = 9
FIRE_BUTTON = 7

# Joint limits
# Not sure if we need this cuz xarm.isValid() should handle it
# in the actual movement part
JOINT_LIMITS = [
    (-180, 180),
    (-108, 114),
    (-123, 92),
    (-180, 180),
    (-100, 123),
    (-180, 180),
]

class Controller(Node):

    def __init__(self):
        super().__init__("controller")
        # sub to /joy
        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.listener_callback, 10)
        
        # pub to /controller_state
        self.controller_state_pub = self.create_publisher(
            Float64MultiArray, "/controller_state", 10)
        
        # pub to /fire
        self.fire_pub = self.create_publisher(
            Bool, '/fire', 10)
        
        timer_period = 0.05  # seconds, which is 20Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.joint_angles = [0.0] * 6

        self.prev_fire_button = 0             # edge detect for fire

        self.leftX = 0.0
        self.rightX = 0.0
        self.dpadX = 0.0

        self.leftY = 0.0
        self.rightY = 0.0
        self.dpadY = 0.0

        self.homeButtonState = 0.0
        self.jointButtonState = 0.0
        self.cartButtonState = 0.0


        
    def listener_callback(self, msg):
        # Extract data from /joy
        axes = msg.axes
        buttons = msg.buttons
        
        self.leftX = axes[LEFT_X]
        self.rightX = axes[RIGHT_X]
        self.dpadX = axes[DPAD_X]

        self.leftY = axes[LEFT_Y]
        self.rightY = axes[RIGHT_Y]
        self.dpadY = axes[DPAD_Y]

        self.homeButtonState = buttons[HOME_IDX]
        self.jointButtonState = buttons[JOINT_BUTTON]
        self.cartButtonState = buttons[CART_BUTTON]

        # Publish controller state
        # controller_state_msg = Float64MultiArray()
        # controller_state_msg.data = [
        #     leftX, rightX, dpadX, leftY, rightY, dpadY, 
        #     homeButtonState, jointButtonState, cartButtonState
        # ]
        # self.controller_state_pub.publish(controller_state_msg)

        # Publish fire
        firing_msg = Bool()
        firing_msg.data = bool(buttons[FIRE_BUTTON])
        
        # If button wasn't already on, and we're now pressing it,
        if self.prev_fire_button == 0 and buttons[FIRE_BUTTON] == 1:
            self.fire_pub.publish(firing_msg)
            self.prev_fire_button = 1

        # If button was being pressed, and now we're not anymore,
        if self.prev_fire_button == 1 and buttons[FIRE_BUTTON] == 0:
            self.fire_pub.publish(firing_msg)
            self.prev_fire_button = 0


    def timer_callback(self):
            controller_state_msg = Float64MultiArray()
            
            controller_state_msg.data = [
                self.leftX, self.rightX, self.dpadX, self.leftY, self.rightY, self.dpadY, 
                self.homeButtonState, self.jointButtonState, self.cartButtonState
            ]
            
            self.controller_state_pub.publish(controller_state_msg)

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Controller()

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()