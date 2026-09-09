#!/usr/bin/env python3
import rclpy
import os
import sys
import threading
import mujoco
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
        self.init_q = None
        self.init_vel = None
        self.model_path = None
        self.robot_data = None
        self.model = None
        self.body_ids = None
        self.ee_site_ids = None
        self.jacp = None 
        self.jacr = None
        self.jacobian = None
        
        #publishers
        self.publisher_state = self.create_publisher(Bool, 'inverse_kinematics_ready', latch_qos)
        self.publisher_result = self.create_publisher(Float64MultiArray, 'inverse_kinematics_result', 10)
        
        #subscriptions
        self.subscription = self.create_subscription(Bool, 'start_running_nodes', self.start_running_nodes_callback, 10)
        self.state_subscription = self.create_subscription(Float64MultiArray, 'init_state_robot', self.state_callback, 10)
        self.init_vel_subscription = self.create_subscription(Float64MultiArray, 'init_vel_robot', self.init_vel_callback, 10)
        self.get_model_path_subscription = self.create_subscription(String, 'model_path', self.get_model_callback, 10)
        
    def start_running_nodes_callback(self, msg):
        if msg.data:
            self.get_logger().info('Starting Inverse Kinematics Node.')
            self.publisher_state.publish(Bool(data=True))
    # The information about the initial state en velocity should be received from the main node or the controller node
    # at de beginning of the simulation or the system.
    def state_callback(self, msg):
        if msg.data:
            self.get_logger().info('Robot is in initial state. Ready to receive target positions.')
            # Here you can add logic to handle the robot's initial state if needed
            self.init_q = np.array(msg.data) 
            
    def init_vel_callback(self, msg):
        if msg.data:
            self.get_logger().info('Robot initial velocity received.')
            # Here you can add logic to handle the robot's initial velocity if needed
            self.init_vel = np.array(msg.data)
    
    def calculate_inverse_kinematics(self, target_position):
        pass
    
    
    def assign_model_data(self, init_q, init_vel):
        if self.model is not None and self.robot_data is not None:
            self.robot_data.qpos[:] = init_q
            self.robot_data.qvel[:] = init_vel
            self.body_ids = {
                self.model.body(i).name: self.model.body(i).id 
                for i in range(self.model.nbody) if self.model.body(i).name
                            }
            self.ee_site_ids = {
                self.model.site(i).name: self.model.site(i).id 
                for i in range(self.model.nsite) if self.model.site(i).name
                               }
            self.get_logger().info('Model data assigned successfully.')

        else:
            self.get_logger().error('Model or robot data is not initialized. Cannot assign data.')
       
        
    def get_model(self, msg):
        if msg.data:
            self.model_path = msg.data
            self.model = mujoco.MjModel.from_xml_path(self.model_path)
            self.robot_data = mujoco.MjData(self.model)
            self.get_logger().info(f'Model path received: {self.model_path}')
            # Eventually, this can be change by a parameter upload at the launch moment.
            # Here you can add logic to load the model if needed
            self.assign_model_data(self.init_q, self.init_vel)