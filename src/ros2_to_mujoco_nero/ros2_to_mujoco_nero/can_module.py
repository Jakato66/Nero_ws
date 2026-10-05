#!/usr/bin/env python3
import can
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, PoseArray

"""This node has to communicate with the CAN bus of agx_arm_ws """

class CanModule(Node):
    def __init__(self):
        super().__init__('can_module')
        self.get_logger().info("CAN module initialized.")

        # parameters
        self._joint_state = JointState()

        # QoS settings

        # publishers
        self.create_publisher(JointState, 'control/joint_states', 10)
        self.create_publisher(JointState, "control/move_j", 10)
        self.create_publisher(PoseStamped, "control/move_p", 10)
        self.create_publisher(PoseStamped, "control/move_l", 10)
        self.create_publisher(PoseArray, "control/move_c", 10)
        self.create_publisher(JointState, "control/move_js", 10)

        # subscribers
        self.test_pos_subscriber = self.create_subscription(Float64MultiArray, 'test_pos', self.test_pos_callback, 10)


    def test_pos_callback(self, msg):
        data = msg.data
        if data == 1:
            self.get_logger().info("Test position 1 received.")
            self._joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
            self._joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            self.get_logger().info("Test position 2 received.")
            self._joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
            self._joint_state.position = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

