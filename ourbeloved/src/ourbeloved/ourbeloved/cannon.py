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

        # Cartesian joystick control
        self.ikIsBusy = False


    # --- Helper Functions ----
    def check_goal(self, goalX, goalY, goalZ):
        # Home position i guess
        goal_joints = [0.0] * 6

        # Error flag will be raised if ik() fails for the next intermediate frame
        error_flag = 0

        # use fk() to get current HTM of arm, and extract current x y z position of end effector
        startJoints = self.xarm.get_joints()
        startHTM, _ = fk(startJoints)
        self.get_logger().info(f"Start Frame x: {startHTM[0, 3]:.2f}, y: {startHTM[1, 3]:.2f}, z: {startHTM[2, 3]:.2f}")

        # this is the goal HTM we want to go to
        goalHTM = startHTM.copy()
        goalHTM[0, 3] = goalX
        goalHTM[1, 3] = goalY
        goalHTM[2, 3] = goalZ

        # makes [x, y, z]        
        startPos = startHTM[:, 3].copy()  # all rows, only col 3
        goalPos = goalHTM[:, 3].copy()

        # currJoints is updated each iteration
        currJoints = startJoints

        numFrames = ITERATIONS  # this will need to be changed to manually calculated

        # create intermediate frames and use ik() to calculate the joints for each intermediate frame. 
        # If ik() fails at any point, we reject the goal request
        for i in range(1, numFrames + 1):
            if (error_flag != 1):  # if no error
                # create intermediate frame
                interPos = startPos + (i / numFrames) * (goalPos - startPos)
                interHTM = startHTM.copy()
                
                interHTM[0, 3] = interPos[0]
                interHTM[1, 3] = interPos[1]
                interHTM[2, 3] = interPos[2]

                # ik() for this intermediate frame
                self.get_logger().info(f"Intermediate Frame x: {interPos[0]:.2f}, y: {interPos[1]:.2f}, z: {interPos[2]:.2f}")
                nextJoints = ik(currJoints, interHTM)

                if nextJoints is None:
                    error_flag = 1
                    self.get_logger().info(f"Error when calculating joints for intermediate frame {i}")
                else:
                    currJoints = nextJoints

        # pass through the final goal frame to check if valid
        if (error_flag == 0):
            error_flag = self.xarm.is_goal_valid(currJoints)
            self.get_logger().info(f"Is goal valid returned: {error_flag}")
            self.get_logger().info(f"Goal Joint Angles: {currJoints}")
        
        # if no error arises in the previous check, it's a valid goal. Accept
        if (error_flag == 0):
            # Valid
            goal_joints = currJoints
            self.get_logger().info("Received goal request")
            return goal_joints
        else:
            # Invalid
            self.get_logger().info("Rejected goal request")
            return None


    def move_to_goal(self, goal_joints, goalX, goalY, goalZ):
        goal_reached = 0  # flag
        isStuck = 0  # flag
        distance = 0.0
        prev_distance = 0.0
        stuck_counter = 0

        # calculate goal HTM
        goalHTM, _ = fk(self.xarm.get_joints())
        goalHTM[0, 3] = goalX
        goalHTM[1, 3] = goalY
        goalHTM[2, 3] = goalZ

        self.xarm.set_joints(goal_joints) # begin going to the goal position

        while (not goal_reached and not isStuck):
            # Calculating distance to goal for feedback and checking if stuck or at goal
            currentHTM, _ = fk(self.xarm.get_joints())

            currentPos = currentHTM[:, 3].copy()  # all rows, only col 3
            goalPos = goalHTM[:, 3].copy()

            diff = currentPos - goalPos
            distance = float(np.sqrt(diff[0]**2 + diff[1]**2 + diff[2]**2))

            # check if stuck or at goal by comparing current distance to previous distance. 
            # If the distance is not decreasing (by at least the stuck threshold), we are stuck. 
            # If the distance is less than the 'at goal threshold', we have reached the goal.

            if (np.abs(prev_distance - distance) < STUCK_THRESHOLD): # this is a jank ass way of doing it. might be better to check the movement of the actual joints for example if the wrist is rotating. 
                stuck_counter += 1
                self.get_logger().info(f"Stuck Counter: {stuck_counter}")
            else:
                stuck_counter = 0

            # Flag stuck
            if (stuck_counter >= STUCK_COUNTER_MAX):
                isStuck = 1
                self.xarm.set_joints(self.xarm.get_joints())   # stay where it is
                self.get_logger().info("I am stuck!")

                return False  # exit out of while loop
            
            # goal reached
            if (distance < GOAL_THRESHOLD):
                goal_reached = 1
                self.get_logger().info("I have reached my destination!")
                
                return True
            
            prev_distance = distance



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

        # MODES ---------
        if self.mode == 'joint':
            # extract controller states from data
            leftX = data[LEFT_X]
            leftY = data[LEFT_Y]
            rightX = data[RIGHT_X]
            rightY = data[RIGHT_Y]
            dpadX = data[DPAD_X]
            dpadY = data[DPAD_Y]

            # get current joints
            currJoints = list(self.xarm.get_joints())

            # increment joint angles based on controller
            currJoints[0] += leftX * JOINT_SPEED
            currJoints[1] += leftY * JOINT_SPEED
            currJoints[2] += rightX * JOINT_SPEED
            currJoints[3] += rightY * JOINT_SPEED
            currJoints[4] += dpadX * JOINT_SPEED
            currJoints[5] += dpadY * JOINT_SPEED

            # if at any point the goal is invalid, stay at currrent position
            if self.xarm.is_goal_valid(currJoints) != 0:
                self.xarm.set_joints(currJoints)


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

            currJoints = self.xarm.get_joints()
            currHTM, _ = fk(currJoints)

            goalHTM = currHTM.copy()
            goalHTM[0, 3] += rightY + CART_SPEED  # X axis, in and out
            goalHTM[1, 3] += rightX + CART_SPEED  # Y axis, left and right
            goalHTM[2, 3] += leftY + CART_SPEED   # Z axis, up and down

            nextJoints = ik(currJoints, goalHTM)

            if nextJoints is not None:  # if ik() gave a joint goal
                if self.xarm.is_goal_valid(nextJoints) == 0:  # if joint goal valid
                    self.xarm.set_joints(nextJoints)

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