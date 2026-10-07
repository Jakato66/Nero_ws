#!/usr/bin/env python3
import can
import rclpy
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
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.gripper_name = "gripper"
        self.init_pose = []

        # QoS settings

        # timers

        # publishers
        self.joint_states_pub = self.create_publisher(JointState, 'control/joint_states', 10)
        self.move_j_pub = self.create_publisher(JointState, "control/move_j", 10)
        self.move_p_pub = self.create_publisher(PoseStamped, "control/move_p", 10)
        self.move_l_pub = self.create_publisher(PoseStamped, "control/move_l", 10)
        self.move_c_pub = self.create_publisher(PoseArray, "control/move_c", 10)
        self.move_js_pub = self.create_publisher(JointState, "control/move_js", 10)
        self.start_node_publisher = self.create_publisher(Bool, '/start_running_nodes', 10)

        # subscribers
        self.create_subscription(Float64MultiArray, 'test_pos', self.test_pos_callback, 10)
        self.create_subscription(Bool, 'start_running_nodes', self.start_node_callback, 10)


    def test_pos_callback(self, msg):
        data = msg.data
        if data and data[0] == 1.0:
            self.get_logger().info("Test position 1 received.")
            self._joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "gripper"]
            self._joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        else:
            self.get_logger().info("Test position 2 received.")
            self._joint_state.name = ["joint1", "joint2", "joint3","joint4", "joint5", "joint6", "joint7", "gripper"]
            self._joint_state.position = [0.5, 0.5, 0.5, 0.5, 10.0, 0.5, 0.5, 0.05]
            effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        msg = JointState()
        msg.name = self._joint_state.name
        msg.position = self._joint_state.position
        msg.effort = effort
        self.move_j_pub.publish(msg)
        self.get_logger().info(f"Published joint states: {msg.position}")

    def start_node_callback(self, msg):
        if msg.data:
            self.get_logger().info('Starting CAN module.')
            msg = Bool()
            msg.data = True
            self.start_node_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    can_module = CanModule()
    rclpy.spin(can_module)
    can_module.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
