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
        
        self.joint_angles = [0.0] * 6

        self.prev_fire_button = 0             # edge detect for fire


        
    def listener_callback(self, msg):
        # Extract data from /joy
        axes = msg.axes
        buttons = msg.buttons
        
        leftX = axes[LEFT_X]
        rightX = axes[RIGHT_X]
        dpadX = axes[DPAD_X]

        leftY = axes[LEFT_Y]
        rightY = axes[RIGHT_Y]
        dpadY = axes[DPAD_Y]

        homeButtonState = buttons[HOME_IDX]
        jointButtonState = buttons[JOINT_BUTTON]
        cartButtonState = buttons[CART_BUTTON]

        # Publish controller state
        controller_state_msg = Float64MultiArray()
        controller_state_msg.data = [
            leftX, rightX, dpadX, leftY, rightY, dpadY, 
            homeButtonState, jointButtonState, cartButtonState
        ]
        self.controller_state_pub.publish(controller_state_msg)

        # Publish fire
        firing_msg = Bool()
        firing_msg.data = bool(buttons[FIRE_BUTTON])
        
        # If button wasn't already on, and we're now pressing it,
        if self.prev_fire_button == 0 and buttons[FIRE_BUTTON] == 1:
            self.fire_pub.publish(firing_msg)

        # If button was being pressed, and now we're not anymore,
        if self.prev_fire_button == 1 and buttons[FIRE_BUTTON] == 0:
            self.fire_pub.publish(firing_msg)




def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Controller()

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()