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
        self.publisher = self.create_publisher(Bool, 'start_running_nodes', 10)
        
        #timers
        self.timer = self.create_timer(self.timer_pr, self.timer_callback)
        
        #subscriptions
        self.subscription = self.create_subscription(Bool, 'mujoco_ready', self.mujoco_ready_callback, 10)
        self.subscription = self.create_subscription(Bool, 'interface_ready', self.interface_ready_callback, 10)
        self.subscription = self.create_subscription(Bool, 'controler_ready', self.controller_ready_callback, 10)
        self.subscription = self.create_subscription(Bool, 'intermediary_ready', self.intermediary_ready_callback, 10)
        
    # callbacks
    def timer_callback(self):
        self.get_logger().info('Main node is running.')
        self.publisher.publish(Bool(data=True))

    def mujoco_ready_callback(self, msg):
        if msg.data:
            self.get_logger().info('Mujoco is ready.')

    def interface_ready_callback(self, msg):
        if msg.data:
            self.get_logger().info('Interface is ready.')

    def controller_ready_callback(self, msg):
        if msg.data:
            self.get_logger().info('Controller is ready.')

    def intermediary_ready_callback(self, msg):
        if msg.data:
            self.get_logger().info('Intermediary is ready.')
            
            
def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode()
    try:
        rclpy.spin(main_node)
    except KeyboardInterrupt:
        pass
    main_node.destroy_node()
    rclpy.shutdown()