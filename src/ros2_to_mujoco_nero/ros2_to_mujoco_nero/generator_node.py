#!/usr/bin/env python3
import rclpy
import os
import numpy as np
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Int8, Float64MultiArray
from geometry_msgs.msg import Pose

"""This node is the trajectory generator"""

class GeneratorNode(Node):
    def __init__(self):
        super().__init__('generator_node')
        self.get_logger().info('Generator node has been started.')

        #qos profiles
        main_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        #atributes
        self.is_ready = False
        self.timer_pr = 0.05
        self.timer = self.create_timer(self.timer_pr, self.generate_callback)
        self.t0 = time.time()
        self.x_center = 0.35
        self.y_center = 0.0
        self.z_center = 0.45
        self.roll = 0.0
        self.pitch = 1.57
        self.yaw = 0.0
        self.amplitude = 0.1
        self.freq = 0.5

        #publishers
        self.ready_publisher = self.create_publisher(Bool, 'generator_ready', main_qos)
        self.obj_position_publisher= self.create_publisher(Float64MultiArray, 'obj_point', main_qos)

        #subscribers
        self.init_subscription = self.create_subscription(Bool, 'start_running_nodes', self.start_callback, main_qos)


    def start_callback(self, msg):
        if msg.data and not self.is_ready:
            self.get_logger().info('Starting the trajectory generator')
            self.is_ready = True
            self.ready_publisher.publish(Bool(data=True))

    def generate_callback(self):
        t = time.time() - self.t0
        x = self.x_center + self.amplitude * np.sin(2 * np.pi * self.freq * t)
        # y = self.y_center + self.amplitude * np.cos(2 * np.pi * self.freq * t)
        
        z = self.z_center
        obj_position_msg = Float64MultiArray()
        obj_position_msg.data = [x, self.y_center, z, self.roll, self.pitch, self.yaw]
        self.obj_position_publisher.publish(obj_position_msg)

def main(args=None):
    rclpy.init(args=args)
    generator_node = GeneratorNode()
    rclpy.spin(generator_node)
    generator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
