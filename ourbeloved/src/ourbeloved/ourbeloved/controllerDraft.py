import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

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


class Controller(Node):

    def __init__(self):
        super().__init__("controller")
        # sub to /joy
        self.joySub = self.create_subscription(
            Joy, "/joy", self.listener_callback, 10)
        
        # pub to /joint_cmd
        self.jointPub = self.create_publisher(
            JointState, "/joint_cmd", 10)
        
        # pub to /cartesian_cmd
        self.cartPub = self.create_publisher(
            JointState, "/cartesian_cmd", 10)
        
        # pub to /homing
        self.homePub = self.create_publisher(
            Bool, "/homing", 10)
        
        # pub to /fire
        self.firePub = self.create_publisher(
            Bool, '/fire', 10)
        
        self.mode = 'joint'         # default is joitn
        self.joint_angles = [0.0] * 6

        self.prevR2 = 0             # edge detect for fire
        self.prevCartButton = 0     # edge detect for cart. mode button
        self.prevJointButton = 0    # edge detect for joint mode button
        self.prevHomeButton = 0     # edge detect for home button    


    def cap(self, val, min, max):
        if val < min:
            return min
        elif val > max:
            return max
        else:
            return val
        
        
    # Subscriber stuff, listening to /joy
    def listener_callback(self, msg):
        # Extract data from /joy
        axes = msg.axes
        buttons = msg.buttons

        # Publisher message types
        jointMSG = JointState()
        homingMSG = Bool()
        firingMSG = Bool()

        # MODE SWITCHING -----
        if buttons[JOINT_BUTTON] == 1 and self.prevJointButton == 0:
            self.mode = 'cartesian'
            self.get_logger().info('Switched to cartestian mode')
        if buttons[CART_BUTTON] == 1 and self.prevCartButton == 0:
            self.mode = 'joint'
            self.get_logger().info('Switched to joint mode')   

        # Update previous button states to whatever it is for this iteration
        self.prevJointButton = buttons[JOINT_BUTTON]
        self.prevCartButton = buttons[CART_BUTTON]


        # HOMING -------
        if buttons[HOME_IDX] == 1 and self.prevHomeButton == 0:
            homingMSG.data = True
            self.homePub.publish(homingMSG)
            self.get_logger().info('Homing request sent...')
        self.prevHomeButton = buttons[HOME_IDX]



        # FIRE -------
        firingMSG.data = bool(buttons[FIRE_BUTTON])
        self.firePub.publish(firingMSG)



        # JOINT MODE ----
        if self.mode == 'joint':
            leftX = axes[LEFT_X]
            leftY = axes[LEFT_Y]
            rightX = axes[RIGHT_X]
            rightY = axes[RIGHT_Y]
            
            jointGoals = [
                leftX * JOINT_SPEED

            ]



        # CARTESIAN MODE -----






def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Controller()

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()