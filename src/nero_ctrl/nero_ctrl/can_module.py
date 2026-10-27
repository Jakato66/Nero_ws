#!/usr/bin/env python3
# Este archivo usa ROS 2 para intercambiar comandos y estados con el controlador
# de AgileX. El módulo "can" se importa, pero aquí no se utilizan sus funciones.
import can
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, PoseArray

# El controlador agx_arm_ctrl, ejecutado como otro nodo, gestiona la interfaz CAN.
# Este programa se comunica con él a través de tópicos ROS 2.
"""This node has to communicate with the CAN bus of agx_arm_ws """

class CanModule(Node):
    def __init__(self):
        super().__init__('can_module')
        self.get_logger().info("CAN module initialized.")

        # Variables utilizadas para preparar comandos y controlar la inicialización.
        # JointState almacena nombres y posiciones de articulaciones.
        # parameters
        self._joint_state = JointState()
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.gripper_name = "gripper"
        # Objetivo inicial: siete posiciones articulares (rad) y apertura
        # del gripper (m), en ese orden.
        self.init_pose = [0.0, -0.40, 0.0, 2.0, 0.0, 0.0, 0.0, 0.05]
        # Estados del arranque y contadores para el envío de la posición inicial.
        # init_pose_ready indica que se envió el comando, no que se alcanzó la pose.
        self.init_pose_ready = False
        self.start_requested = False
        self.feedback_ready = False
        self.initial_pose_attempts = 0
        self.initial_pose_first_sent_at = None


        # QoS determina cómo ROS 2 conserva y entrega los mensajes.
        # TRANSIENT_LOCAL conserva el último mensaje para suscriptores compatibles;
        # VOLATILE no conserva mensajes anteriores para nuevos suscriptores.
        # QoS settings
        self.init_node_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ik_qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        # timers

        # Publicadores: envían mensajes a otros nodos a través de tópicos.
        # Los dos primeros publican en control/move_j, con QoS diferentes:
        # uno para el comando de inicio y otro para movimientos posteriores.
        # publishers
        # publishers of agx_arm_ctrl_single_node.py
        self.init_state_joint_pub = self.create_publisher(JointState, 'control/move_j', self.init_node_qos)
        self.move_j_pub = self.create_publisher(JointState, "control/move_j", 10)
        # Se preparan otros modos de movimiento del controlador AgileX.
        # En este archivo se crean, pero no se utilizan para publicar comandos.
        self.move_p_pub = self.create_publisher(PoseStamped, "control/move_p", 10)
        self.move_l_pub = self.create_publisher(PoseStamped, "control/move_l", 10)
        self.move_c_pub = self.create_publisher(PoseArray, "control/move_c", 10)
        self.move_js_pub = self.create_publisher(JointState, "control/move_js", 10)
        
        # Publica la señal de inicio y los estados articulares destinados a IK.
        # others publishers (can modify and use this)
        self.start_node_publisher = self.create_publisher(Bool, '/start_running_nodes', self.init_node_qos)
        
        self.joint_state_publisher_for_ik = self.create_publisher(
            JointState,
            '/ik/joint_states',
            ik_qos,
        )
        
        # Suscriptores: vinculan cada tópico recibido con su función callback.
        # Reciben pruebas, inicio, realimentación del robot y resultados de IK.
        # subscribers
        self.create_subscription(Float64MultiArray, 'test_pos', self.test_pos_callback, 10)
        self.create_subscription(Bool, 'start_running_nodes', self.start_node_callback, 10)
        self.create_subscription(JointState, 'feedback/joint_states', self.joint_states_callback, ik_qos)
        self.create_subscription(JointState, 'ik/ik_result', self.ik_result_callback, ik_qos)
        # Comprueba cada 0,25 segundos si se puede enviar la posición inicial.
        self.initial_pose_timer = self.create_timer(0.25, self._publish_initial_pose_when_ready)


    # Publica una de dos posiciones articulares de prueba.
    # La selección depende del primer número recibido en test_pos.
    def test_pos_callback(self, msg):
        data = msg.data
        # Si el primer elemento es exactamente 1.0, selecciona la primera pose.
        if data and data[0] == 1.0:
            self.get_logger().info("Test position 1 received.")
            self._joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "gripper"]
            self._joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        # Cualquier otro contenido, incluso un arreglo vacío, elige la segunda.
        else:
            self.get_logger().info("Test position 2 received.")
            self._joint_state.name = ["joint1", "joint2", "joint3","joint4", "joint5", "joint6", "joint7", "gripper"]
            # Se conserva el valor original de joint5 (10.0).
            # No implica que sea un objetivo seguro o dentro de los límites físicos.
            self._joint_state.position = [0.5, 0.5, 0.5, 0.5, 10.0, 0.5, 0.5, 0.05]
            effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        # Construye el comando JointState y lo envía a control/move_j.
        # effort incluye la fuerza solicitada para el gripper en su última posición.
        msg = JointState()
        msg.name = self._joint_state.name
        msg.position = self._joint_state.position
        msg.effort = effort
        self.move_j_pub.publish(msg)
        self.get_logger().info(f"Published joint states: {msg.position}")

    # Recibe una señal booleana de inicio. La retransmite solamente la primera
    # vez que recibe True, evitando volver a publicarla en cada recepción.
    def start_node_callback(self, msg):
        if msg.data and not self.start_requested:
            self.start_requested = True
            self.get_logger().info('Starting CAN module.')
            self.start_node_publisher.publish(Bool(data=True))

    # Temporizador que coordina el envío de la pose inicial.
    # Requiere tanto la orden de inicio como datos articulares recibidos.
    def _publish_initial_pose_when_ready(self):
        # Si falta alguna condición, termina esta ejecución del temporizador.
        if not self.start_requested or not self.feedback_ready:
            return

        # Reloj monotónico: mide intervalos sin depender de la hora del sistema.
        # La primera vez que se cumplen las condiciones se fija el instante base.
        now = time.monotonic()
        if self.initial_pose_first_sent_at is None:
            self.initial_pose_first_sent_at = now
            return

        # Espera al menos un segundo desde el instante base antes del primer envío.
        if now - self.initial_pose_first_sent_at < 1.0:
            return
        # Tras tres publicaciones se cancela este temporizador.
        if self.initial_pose_attempts >= 3:
            self.initial_pose_timer.cancel()
            return
        # Aplica una condición temporal adicional a los reintentos,
        # calculada desde el mismo instante base de la inicialización.
        if self.initial_pose_attempts and now - self.initial_pose_first_sent_at < 0.5 * (self.initial_pose_attempts + 1):
            return

        # Envía las siete articulaciones y el gripper como comando MOVE J.
        # El último valor de effort corresponde al gripper.
        msg = JointState()
        msg.name = self.joint_names + [self.gripper_name]
        msg.position = self.init_pose
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.init_state_joint_pub.publish(msg)
        self.initial_pose_attempts += 1
        # Se marca como enviada la pose inicial, sin confirmar movimiento físico.
        self.init_pose_ready = True
        self.get_logger().info(
            f"Published initial joint states (attempt {self.initial_pose_attempts}/3): {msg.position}"
        )

    
    # Recibe los estados articulares publicados por el controlador AgileX.
    # Después del envío inicial, reenvía nombres y posiciones al nodo IK.
    def joint_states_callback(self, msg):
        # Considera disponible la realimentación cuando llegan al menos
        # siete posiciones; no comprueba aquí si el robot llegó al objetivo.
        if len(msg.position) >= len(self.joint_names):
            self.feedback_ready = True
        # Solo se copian los campos name y position: los campos velocity,
        # effort y header del mensaje recibido no se transfieren en este bloque.
        if self.init_pose_ready:
            self._joint_state.name = msg.name
            self._joint_state.position = msg.position
            self.joint_state_publisher_for_ik.publish(self._joint_state)

    # Recibe el resultado de cinemática inversa y lo retransmite como
    # comando de movimiento articular al controlador AgileX, sin modificarlo.
    def ik_result_callback(self, msg):
        self.get_logger().info(f"Received IK result: {msg.position}")
        # initially only with movej
        self.move_j_pub.publish(msg)

# Punto de entrada: inicia ROS 2, crea el nodo y procesa callbacks hasta
# detener la ejecución; luego destruye el nodo y cierra ROS 2.
def main(args=None):
    rclpy.init(args=args)
    can_module = CanModule()
    rclpy.spin(can_module)
    can_module.destroy_node()
    rclpy.shutdown()

# Ejecuta main() solo cuando se inicia este archivo directamente.
if __name__ == '__main__':
    main()
