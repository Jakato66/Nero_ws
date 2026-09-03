#!/usr/bin/env python3
import rclpy
import os
import sys
import threading
import mujoco
import mujoco.viewer
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import JointState

"""This node handle the mujoco simulator"""


class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        self.get_logger().info('Mujoco node has been started.')
        
        # atributes
        self.timer_pr = 0.01 # 100Hz
        
        # publishers
        self.publisher = self.create_publisher(Bool, 'mujoco_ready', 10)
        
        #subscriptions
        self.subscription = self.create_subscription(Bool, 'start_running_nodes', self.start_running_nodes_callback, 10)
        
        #threads
        self.mujoco_thread = threading.Thread(target=self.run_mujoco_simulator)
        
    def start_running_nodes_callback(self, msg):
        if msg.data:
            self.get_logger().info('Starting Mujoco simulator.')
            self.publisher.publish(Bool(data=True))
            # Start the mujoco simulator in a separate thread
            
            self.mujoco_thread.start()
    
    def run_mujoco_simulator(self):
        # # Verify that mujoco venv is active
        # if "VIRTUAL_ENV" not in os.environ:
        #     self.get_logger().error("Mujoco venv is not active. Please activate the mujoco venv before running this node.")
        #     # Send exit message to main node
            
        #     # Exit
        #     sys.exit(1)
        
        xml_path = os.path.expanduser('~/Projects/iPre_nero/src/models_for_simulation/agilex_nero/scene.xml')
        
        try:
            model = mujoco.MjModel.from_xml_path(xml_path)
            data = mujoco.MjData(model)
            
            # Launch mujoco viewer
            self.get_logger().info('Launching Mujoco viewer.')
            mujoco.viewer.launch(model, data)

        except Exception as e:
            self.get_logger().error(f"Failed to load Mujoco model or launching viewer from {xml_path}: {e}")
            sys.exit(1)
            
def main(args=None):
    rclpy.init(args=args)
    mujoco_node = MujocoNode()
    try:
        rclpy.spin(mujoco_node)
    except KeyboardInterrupt:
        pass
    mujoco_node.mujoco_thread.join()  # Wait for the mujoco thread to finish before exiting
    mujoco_node.destroy_node()
    rclpy.shutdown()
    
# Mensaje de prueba para levantar el nodo de manera individual
# ros2 topic pub /start_running_nodes std_msgs/msg/Bool "{data: true}" -1
# publisher: beginning loop
# publishing #1: std_msgs.msg.Bool(data=True)
