#!/usr/bin/env python3

import rclpy
import os
import time
import mujoco
import mujoco.viewer
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import JointState

"""This node handle the mujoco simulator"""


class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        self.get_logger().info('Mujoco node has been started.')
        
         # QoS profile 
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                
        # atributes
        self.timer_pr = 0.01 # 100Hz
        self.start_requested = False
        # This may change if needed
        self.model_path = os.path.expanduser('~/Projects/iPre_nero/src/models_for_simulation/agilex_nero/scene.xml')
        
        # publishers
        self.publisher_state_ready = self.create_publisher(Bool, '/mujoco_ready', latch_qos)
        self.publisher_joint_state = self.create_publisher(JointState, '/joint_states', latch_qos)
        
        #subscriptions
        self.subscription = self.create_subscription(Bool, '/start_running_nodes', self.start_running_nodes_callback, 10)
        
    def start_running_nodes_callback(self, msg):
        if msg.data and not self.start_requested:
            self.get_logger().info('Starting Mujoco simulator.')
            self.publisher_state_ready.publish(Bool(data=True))
            # Start the mujoco simulator 
            self.start_requested = True
            
def main(args=None):
    rclpy.init(args=args)
    mujoco_node = MujocoNode()
    try:
        model = mujoco.MjModel.from_xml_path(mujoco_node.model_path)
        data = mujoco.MjData(model)

        mujoco_node.get_logger().info(
            'Waiting for /start_running_nodes...'
        )

        while rclpy.ok() and not mujoco_node.start_requested:
            rclpy.spin_once(mujoco_node, timeout_sec=0.1)

        if not rclpy.ok():
            return

        mujoco_node.get_logger().info(
            'Start message received. Launching MuJoCo.'
        )

        with mujoco.viewer.launch_passive(model, data) as viewer:
            mujoco_node.publisher_state_ready.publish(Bool(data=True))

            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(mujoco_node, timeout_sec=0.0)

                mujoco.mj_step(model, data)
                viewer.sync()

                time.sleep(model.opt.timestep)
    except Exception as e:
        mujoco_node.get_logger().error(f"Failed to load Mujoco model or launch viewer: {e}")
       
    finally:
        mujoco_node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
    
# Mensaje de prueba para levantar el nodo de manera individual
# ros2 topic pub /start_running_nodes std_msgs/msg/Bool "{data: true}" -1
# publisher: beginning loop
# publishing #1: std_msgs.msg.Bool(data=True)
