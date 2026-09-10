#!/usr/bin/env python3
import rclpy
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
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
        self.start_sent = False
        self.model_path = os.path.expanduser('~/Projects/iPre_nero/src/models_for_simulation/agilex_nero/scene.xml')
        model_path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # publishers
        self.publisher = self.create_publisher(
            Bool,
            'start_running_nodes',
            model_path_qos,
        )
        self.publisher_model_path = self.create_publisher(
            String,
            '/model_path',
            model_path_qos,
        )

        model_path_msg = String()
        model_path_msg.data = self.model_path
        self.publisher_model_path.publish(model_path_msg)
        
        # Publish the start command once, after the launched nodes have had
        # time to create their subscriptions.
        self.start_timer = self.create_timer(0.5, self.publish_start_once)

        #subscriptions
        self.subscription = self.create_subscription(Bool, 'mujoco_ready', self.mujoco_ready_callback, model_path_qos)
        self.subscription = self.create_subscription(Bool, 'interface_ready', self.interface_ready_callback, model_path_qos)
        self.subscription = self.create_subscription(Bool, 'controler_ready', self.controller_ready_callback, model_path_qos)
        self.subscription = self.create_subscription(Bool, 'intermediary_ready', self.intermediary_ready_callback, model_path_qos)
        
    # callbacks
    def publish_start_once(self):
        if self.start_sent:
            return

        self.publisher.publish(Bool(data=True))
        self.start_sent = True
        self.start_timer.cancel()
        self.get_logger().info('Published /start_running_nodes once.')

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
    main_node.get_logger().info('Main node is running.')
    try:
        rclpy.spin(main_node)
    except KeyboardInterrupt:
        pass
    main_node.destroy_node()
    rclpy.shutdown()