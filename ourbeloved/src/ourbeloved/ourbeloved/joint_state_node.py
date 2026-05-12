import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState
from xarmclient import XArm
import math

class JointStateNode(Node):

    def __init__(self):
        super().__init__("joint_state_node")
        self.publisher = self.create_publisher(JointState, "/joint_state", 10)
        timer_period = 0.2  # seconds, which is 5Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.xarm = XArm()

        self.joint_names = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
        self.joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # degrees

    def timer_callback(self):
        msg = JointState()

        # TODO: Header???
        
        # for joint in range(self.xarm.get_joints()):
        self.joint_pos= list(self.xarm.get_joints())

        for i in range(len(self.joint_pos)):
            self.joint_pos[i] = (self.joint_pos[i] * math.pi) / 180

        msg.name = self.joint_names
        msg.position = self.joint_pos
        #msg.velocity = []
        #msg.effort = []

        self.publisher.publish(msg)


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = JointStateNode()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()