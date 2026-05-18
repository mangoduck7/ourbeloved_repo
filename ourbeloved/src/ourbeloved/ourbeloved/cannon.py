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

from copy import deepcopy

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
        target_joints = msg.position

        # If the given goal is invalid, exit out of precise mode
        if self.xarm.is_goal_valid(target_joints) != 0:
            self.get_logger().info('Joint goal not valid') 
            self.precise_mode = False
            return None
        
        # VARIABLES -----
        switch2PI_threshold = 10 
        precise_threshold = 0.1
        INCREMENT_VAL = 0.05

        keepGoing = 1   # while loop flag for initial set joints
        keepGoing2 = 1  # while loop flag for incremental joint angle changes

        totalDiff = 0   # for comparing to stuck_threshold

        stuck_threshold = 0.1
        stuckCounter = 0
        isStuck = 0

        distance = [0.0] * 6        # distance to goal
        diffPrevCurr = [0.0] * 6    # for checking if stuck
        prevJoints = [0.0] * 6      # for checking if stuck


        # Tell the robot to go to the goal joints
        self.xarm.set_joints(target_joints)
        self.where_i_should_be = deepcopy(target_joints)  # used in PI controller

        while keepGoing and not isStuck:
            currJoints = list(self.xarm.get_joints())
            time.sleep(self.timer_period)  # 20Hz operation

            # Reset these vals after each operation
            keepGoing = 0       
            totalDiff = 0.0

            for i in range(6):
                # Compare current joint pos to target joint pos
                distance[i] = abs(currJoints[i] - target_joints[i])

                # Compare current joint pos to previous
                diffPrevCurr[i] = abs(currJoints[i] - prevJoints[i])
                totalDiff += diffPrevCurr[i]

                # If difference for ALL joints is < 10, we're close enough
                if distance[i] > switch2PI_threshold:
                    keepGoing = 1   

            # STUCK LOGIC -----
            if totalDiff < stuck_threshold:
                stuckCounter += 1
                self.get_logger().info(f'Stuck counter: {stuckCounter}')
            else:
                stuckCounter = 0

            if stuckCounter >= 100:
                isStuck = 1
                self.get_logger().info(f'Stuck at a totalDiff of: {totalDiff}')

            prevJoints = currJoints

        # (end while loop)

        self.get_logger().info('Switched to PI controller...')
        velocities = [INCREMENT_VAL / self.timer_period] * 6

        # "PI Controller" ---------
        current_joint_idx = 0 
        hold_counter = 0
        hold_required = int(2.0 / self.timer_period)  # 2 seconds 

        while keepGoing2:
            currJoints = list(self.xarm.get_joints())
            time.sleep(self.timer_period)  # 20 Hz

            nextJoints = self.where_i_should_be
            all_in_threshold = True

            for i in range(6):
                error = target_joints[i] - currJoints[i]

                # if this joint is done, move on to next
                if abs(error) > precise_threshold:
                    all_in_threshold = False
                    self.get_logger().info(f'error: {error} for joint: {i}')

                    # if currJoints not at targetJoints yet, move more
                    if error > 0:   
                        increment = INCREMENT_VAL
                    # if currJoints overshot targetJoints, move back
                    else:           
                        increment = -INCREMENT_VAL
                    
                    nextJoints[i] += increment
                else:
                    velocities[i] = 0.0

            # if increment is valid, move incrementally
            if self.xarm.is_goal_valid(nextJoints) == 0:    
                self.xarm.set_joints(nextJoints, "high_acc", velocities)
                self.where_i_should_be = nextJoints  # do we need this? or can we call get_joints here
                # self.get_logger().info(f'where i should be: {self.where_i_should_be}')
                # self.get_logger().info(f'current joints: {currJoints}')
                # self.get_logger().info(f'target joints: {target_joints}')

            if all_in_threshold:
                hold_counter += 1
                self.get_logger().info(f'Hold Counter: {hold_counter}')
                self.get_logger().info(f'get joints: {self.xarm.get_joints()}')
            else:
                hold_counter = 0

            if hold_counter >= hold_required:
                self.get_logger().info('Precise goal reached.')
                self.get_logger().info(f'get joints: {self.xarm.get_joints()}')
                keepGoing2 = False

        self.precise_mode = False  # exit precise mode at the end of function callback



def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Cannon()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()