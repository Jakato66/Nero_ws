#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int8
"""This node has to be started after the ros2_to_mujoco_nero package has been 
launched. It will start the main loop of the simulation and will also ensure 
that all the nodes ar up and running"""

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.get_logger().info('Main node has been started.')
        
        # atributes
        self.timer_pr = 0.01 # 100Hz
        
        # publishers
        self.publishers = self.create_publisher(Bool, 'start_simulation', 10)
        
        #timers
        self.timer = self.create_timer(self.timer_pr, self.timer_callback)
        
        #subscriptions
        