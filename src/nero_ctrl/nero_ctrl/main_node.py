#!/usr/bin/env python3
# DOCUMENTACIÓN: Nodo coordinador: difunde ruta del modelo, arranque y objetivo cartesiano.
import rclpy
import os
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Int8, Float64MultiArray
from geometry_msgs.msg import Pose
"""This node has to be started after the ros2_to_mujoco_nero package has been 
launched. It will start the main loop of the simulation and will also ensure 
that all the nodes ar up and running"""

# Centraliza la señal de arranque, la ruta del modelo y los objetivos de posición.
class MainNode(Node):
    # Declara los publicadores/suscriptores y programa el mensaje de inicio único.
    def __init__(self):
        super().__init__('main_node')
        self.get_logger().info('Main node has been started.')
        
        # atributes
        self.timer_pr = 0.01 # 100Hz
        self.start_sent = False
        # La ruta del modelo es fija y depende de la carpeta usada en el computador original.
        self.model_path = os.path.expanduser('~/Projects/iPre_nero/src/models_for_simulation/agilex_nero/scene.xml')
        model_path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        input_obj_qos = QoSProfile(depth=10)
        
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
        self.publisher_obj_position = self.create_publisher(Float64MultiArray, '/obj_point', model_path_qos)

        # Anuncia el archivo XML para los nodos que necesitan cargar el modelo MuJoCo.
        model_path_msg = String()
        model_path_msg.data = self.model_path
        self.publisher_model_path.publish(model_path_msg)
        
        # Publish the start command once, after the launched nodes have had
        # time to create their subscriptions.
        self.start_timer = self.create_timer(0.5, self.publish_start_once)

        #subscriptions
        self.subscription_mujoco = self.create_subscription(Bool, 'mujoco_ready', self.mujoco_ready_callback, model_path_qos)

        self.subscription_controller = self.create_subscription(Bool, 'controller_ready', self.controller_ready_callback, model_path_qos)
        self.subscription_ik = self.create_subscription(Bool, 'inverse_kinematics_ready', self.ik_ready_callback, model_path_qos)
        #this topic is to receive the goal position
        # Recibe un objetivo cartesiano; este nodo solo lo retransmite.
        self.subscription_obj_position = self.create_subscription(Float64MultiArray, 'input_obj_position', self.input_obj_position_callback, input_obj_qos)

        self.subscription_can = self.create_subscription(Bool, 'can_ready', self.can_ready_callback, model_path_qos)
        
    # callbacks
    # Publica True una sola vez y cancela el temporizador de inicio.
    def publish_start_once(self):
        if self.start_sent:
            return

        self.publisher.publish(Bool(data=True))
        self.start_sent = True
        self.start_timer.cancel()
        self.get_logger().info('Published /start_running_nodes once.')

    # Registra la confirmación de que el simulador está disponible.
    def mujoco_ready_callback(self, msg):
        if msg.data:
            self.get_logger().info('Mujoco is ready.')
    # Registra la disponibilidad del nodo de cinemática inversa.
    def ik_ready_callback(self, msg):
        if msg.data:
            self.get_logger().info('Inverse kinematics is ready.')

    # Registra la confirmación de disponibilidad del controlador.
    def controller_ready_callback(self, msg):
        if msg.data:
            self.get_logger().info('Controller is ready.')
    
    # Registra la confirmación de disponibilidad del módulo CAN.
    def can_ready_callback(self, msg):
        if msg.data:
            self.get_logger().info('CAN is ready.')


    # Reenvía el objetivo recibido a /obj_point sin transformarlo.
    def input_obj_position_callback(self, msg):
        self.get_logger().info(f"Received input object position: {msg.data}")
        self.publisher_obj_position.publish(msg)
        # [x, y, z, roll, pitch, yaw]


# Crea el nodo ROS 2 y procesa mensajes hasta su finalización.
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

if __name__ == '__main__':
    main()
