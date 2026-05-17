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

import numpy as np
import time

# index from controller_state array
# [leftX, rightX, dpadX, leftY, rightY, dpadY, 
# homeButtonState, jointButtonState, cartButtonState]

# Variables
JOINT_DELTA_MULTIPLIER = 2.0  # degrees per iteration
CART_DELTA_MULTIPLIER = 5.0   # mm per iteration

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
            self.controller_state_callback, 10)
        
        self.precise_joint_sub = self.create_subscription(
            JointState, 
            "/precise_joint_cmd", 
            self.precise_joint_callback, 10)
        
        # Robot
        self.xarm = XArm()
        self.where_i_should_be = [0.0, 45.0, -80.0, 0.0, 35.0, 0.0]

        # Modes
        self.mode = 'joint'
        self.precise_mode = False

        # Button press detection
        self.prev_home_button = 0
        self.prev_joint_button = 0
        self.prev_cart_button = 0

        # Timer (generic template)
        self.timer_period = 0.05  # seconds, make sure this is the same as controller
        #self.timer = self.create_timer(timer_period, self.timer_callback)

        # Cartesian joystick control
        self.ikIsBusy = False

    


    def controller_state_callback(self, msg):
        # if we're meant to be in precise mode, exit out of function
        if self.precise_mode == True:
            return None
        
        data = msg.data

        #self.get_logger().info(f'data received lx: {data[LEFT_X]}, ly: {data[LEFT_Y]}, rx: {data[RIGHT_X]}, ry: {data[RIGHT_Y]}, dpx: {data[DPAD_X]}, dpy: {data[DPAD_Y]}')  

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
            self.where_i_should_be = [0.0, 45.0, -80.0, 0.0, 35.0, 0.0]
            self.get_logger().info('Homing...')
            time.sleep(2)
        self.prev_home_button = data[HOME_BUTTON]  # update state

        # MODES ---------
        if self.mode == 'joint':
            # extract controller states from data
            leftX = data[LEFT_X]
            leftY = data[LEFT_Y]
            rightX = data[RIGHT_X]
            rightY = data[RIGHT_Y]
            dpadX = data[DPAD_X]
            dpadY = data[DPAD_Y]

            # if no joystick inputs, skip this iteration
            if leftX == 0.0 and leftY == 0.0 and rightX == 0.0 and rightY == 0.0 and dpadX == 0.0 and dpadY == 0.0:
                return None

            # get current joints
            currJoints = list(self.where_i_should_be)   #previously: list(self.xarm.get_joints())

            # increment joint angles based on controller
            currJoints[0] += leftX * JOINT_DELTA_MULTIPLIER
            currJoints[1] += leftY * JOINT_DELTA_MULTIPLIER
            currJoints[2] += rightY * JOINT_DELTA_MULTIPLIER
            currJoints[3] += rightX * -JOINT_DELTA_MULTIPLIER
            currJoints[4] += dpadY * JOINT_DELTA_MULTIPLIER
            currJoints[5] += dpadX * -JOINT_DELTA_MULTIPLIER

            # Calculate joint velocities (degrees/second)
            joint1_vel = (leftX * JOINT_DELTA_MULTIPLIER) / self.timer_period
            joint2_vel = (leftY * JOINT_DELTA_MULTIPLIER) / self.timer_period
            joint3_vel = (rightY * JOINT_DELTA_MULTIPLIER) / self.timer_period
            joint4_vel = (rightX * JOINT_DELTA_MULTIPLIER) / self.timer_period
            joint5_vel = (dpadY * JOINT_DELTA_MULTIPLIER) / self.timer_period
            joint6_vel = (dpadX * JOINT_DELTA_MULTIPLIER) / self.timer_period

            joint_velocities = [joint1_vel, joint2_vel, joint3_vel, joint4_vel, joint5_vel, joint6_vel]

            # If goal is valid, then move based on joystick input
            if self.xarm.is_goal_valid(currJoints) == 0:  # valid
                self.where_i_should_be = currJoints
                self.xarm.set_joints(list(self.where_i_should_be), "high_acc", list(joint_velocities))
            else:
                self.get_logger().info('Joint goal not valid')  


        elif self.mode == 'cartesian':
            # if an ik() calculation is ongoing, skip this iteration to let it process
            if self.ikIsBusy == True:
                return None
            
            rightX = data[RIGHT_X]  # Y axis (left and right)
            rightY = data[RIGHT_Y]  # X axis (in and out)
            leftY = data[LEFT_Y]    # Z axis (up and down)
            
            # if no joystick input, just don't move
            if rightX == 0.0 and rightY == 0.0 and leftY == 0.0:
                return None
            
            # start ik() if there IS joystick input
            self.ikIsBusy = True  

            currJoints = list(self.where_i_should_be) #self.xarm.get_joints()
            currHTM, _ = fk(currJoints)

            ee_deltas = [rightY*CART_DELTA_MULTIPLIER, rightX*CART_DELTA_MULTIPLIER, leftY*CART_DELTA_MULTIPLIER, 1]

            goalHTM = currHTM.copy()

            goalPos = np.vstack([currHTM, np.array([0, 0, 0, 1])]) @ np.array(ee_deltas)

            goalHTM[0, 3] = goalPos[0]  # X axis, in and out
            goalHTM[1, 3] = goalPos[1]  # Y axis, left and right
            goalHTM[2, 3] = goalPos[2]  # Z axis, up and down

            nextJoints = ik(currJoints, goalHTM)
            #self.get_logger().info(f'Next joints: {nextJoints}')  

            if nextJoints is not None:  # if ik() gave a joint goal
                if self.xarm.is_goal_valid(nextJoints) == 0:  # if joint goal valid
                    joint_velocities = (nextJoints - currJoints) / self.timer_period
                    self.where_i_should_be = nextJoints
                    joint_velocities = (np.array(nextJoints) - np.array(currJoints)) / self.timer_period
                    self.xarm.set_joints(list(self.where_i_should_be), "high_acc", joint_velocities)
                else:
                    self.get_logger().info('Joint goal not valid') 
            else:
                self.get_logger().info('Joint goal diverged')  

            self.ikIsBusy = False


    def precise_joint_callback(self, msg):
        self.precise_mode = True
        joint_pos_list = msg.position



def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Cannon()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()