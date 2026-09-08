#!/usr/bin/env python3
import rclpy
import os
import sys
import threading
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import JointState


"""This nodes handle the inverse kinematics calculation and send the reasult to 
the controller node and then to the indermediary node"""

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.get_logger().info('Inverse Kinematics Node has been started.')
        
        # QoS profile 
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # atributes 
        self.timer_pr = 0.01 # 100Hz
        
        #publishers
        self.publisher_state = self.create_publisher(Bool, 'inverse_kinematics_ready', latch_qos)
        self.publisher_result = self.create_publisher(Float64MultiArray, 'inverse_kinematics_result', 10)
        
        #subscriptions
        self.subscription = self.create_subscription(Bool, 'start_running_nodes', self.start_running_nodes_callback, 10)
        
    def start_running_nodes_callback(self, msg):
        if msg.data:
            self.get_logger().info('Starting Inverse Kinematics Node.')
            self.publisher_state.publish(Bool(data=True))

    def calculate_inverse_kinematics(self, target_position):
        pass