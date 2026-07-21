import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from utils import load_trajectory_from_txt, calculate_3d_distance, calculate_trajectory_rate
import numpy as np
import time

# Mensajes de MAVROS en ROS2
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class VincentVanDroneNode(Node):
    def __init__(self):
        super().__init__('vincent_van_drone_multirun')
        
        # --- 1. CONFIGURACIONES (Reemplazaría tu CONFIGS_MULTIRUN_ROS.m) ---
        self.pub_rate = 25.0  # Hz
        self.takeoff_alt = 0.652    
        self.start_pos = [-1.049, 2.89, self.takeoff_alt] # Vértice inferior izquierdo
        
        # --- 2. ESTADOS Y VARIABLES ---
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.is_trajectory_loaded = False
        self.mission_state = "INIT" # Máquina de estados: INIT -> ARM -> TAKEOFF -> GOTO_START -> PUBLISH -> LAND
        self.warmup_counter = 0

        # --- 3. QOS PROFILES (ROS2 requiere configurar la calidad del servicio) ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_services = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        # --- 4. SUBSCRIBERS ---
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile)

        # --- 5. PUBLISHERS ---
        # Para ir a la posición inicial (WP)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Ejemplo: Publisher RAW (equivalente a tu SIM_ROS_Raw_Publisher.m)
        self.raw_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

        # --- 6. CLIENTES DE SERVICIOS (Armar, Modos, Despegar) ---
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        # --- 7. BUCLE PRINCIPAL (Timer) ---
        # En ROS2 no hacemos bucles while bloqueantes, usamos un Timer que se ejecuta a N Hz
        self.timer_period = 1.0 / self.pub_rate
        self.timer = self.create_timer(self.timer_period, self.mission_loop_cb)
        
        # Variables de trayectoria
        self.traj_points = []
        self.current_traj_idx = 0
        
        self.get_logger().info("Vincent Van Drone Node Inicializado en ROS2.")
        self.load_trajectory() # Llama a tu función de utils.py

    # --- CALLBACKS DE SENSORES ---
    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    # --- CARGA DE DATOS ---
    def load_trajectory(self):
        self.get_logger().info("Cargando matriz de waypoints (Python txt load)...")
        data = load_trajectory_from_txt('../trayectorias_exportadas/20260720-135951_trajectory_ros2')
        if data is not None:
            self.traj_points = data['pos']
            self.traj_vels = data['vel']
            self.traj_accs = data['acc']
            self.is_trajectory_loaded = True
            self.get_logger().info("Trayectoria cargada y lista para publicar.")
        else:
            self.get_logger().error("Falló la carga de la trayectoria.")
        # self.traj_points = np.loadtxt(...)
        self.is_trajectory_loaded = True

    # --- MÁQUINA DE ESTADOS PRINCIPAL ---
    def mission_loop_cb(self):
        # MAVROS requiere que enviemos setpoints continuamente ANTES de cambiar a modo Offboard
        if self.mission_state in ["INIT", "ARM", "TAKEOFF", "GOTO_START"]:
            self.publish_hover_setpoint(self.start_pos)

        if not self.current_state.connected:
            return # Esperamos a que MAVROS conecte con PX4

        # 1. ARMADO Y OFFBOARD
        if self.mission_state == "INIT":
            self.warmup_counter += 1
            
            # Esperamos 50 ciclos (2 segundos) publicando setpoints VÁLIDOS
            if self.warmup_counter > 50:
                if self.current_state.mode != "OFFBOARD":
                    # 1º Paso: Pedir modo OFFBOARD (ahora que las coordenadas tienen hora válida)
                    if self.warmup_counter % 25 == 0:
                        self.get_logger().info("Solicitando modo OFFBOARD...")
                        self.set_offboard_mode()
                        
                elif not self.current_state.armed:
                    # 2º Paso: Una vez aceptado el modo OFFBOARD, armar motores
                    if self.warmup_counter % 25 == 0:
                        self.get_logger().info("Solicitando ARM (Armar motores)...")
                        self.arm_drone()
                        
                else:
                    self.get_logger().info("Dron Armado y en Offboard. Volando a posición inicial...")
                    self.mission_state = "GOTO_START"
            else:
                if self.warmup_counter % 10 == 0:
                    self.get_logger().info(f"Calentando conexión OFFBOARD: {self.warmup_counter}/50...")
        
        # 2. IR A POSICIÓN DE INICIO Y HOVER
        elif self.mission_state == "GOTO_START":
            # Calcular distancia al objetivo
            dx = self.current_pose.pose.position.x - self.start_pos[0]
            dy = self.current_pose.pose.position.y - self.start_pos[1]
            dz = self.current_pose.pose.position.z - self.start_pos[2]
            dist = np.sqrt(dx**2 + dy**2 + dz**2)

            if dist < 0.1: # Tolerancia (distance_goal_reached en tu config de MATLAB)
                self.get_logger().info("Posición inicial alcanzada. Empezando trayectoria...")
                self.mission_state = "PUBLISH_TRAJECTORY"
                self.current_traj_idx = 0

        # 3. PUBLICAR TRAYECTORIA (Equivalente al bucle for de tus scripts SIM_ROS)
        elif self.mission_state == "PUBLISH_TRAJECTORY":
            if self.current_traj_idx < len(self.traj_points):
                msg_raw = PositionTarget()
                msg_raw.header.stamp = self.get_clock().now().to_msg()
                msg_raw.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                
                # Máscara 3520 (Ignora aceleración/yaw, hace caso a Posición + Velocidad)
                msg_raw.type_mask = 3072
                
                # Asignar Posiciones (q.txt)
                msg_raw.position.x = float(self.traj_points[self.current_traj_idx][0])
                msg_raw.position.y = float(self.traj_points[self.current_traj_idx][1])
                msg_raw.position.z = float(self.traj_points[self.current_traj_idx][2])
                
                # Asignar Velocidades (qd.txt)
                msg_raw.velocity.x = float(self.traj_vels[self.current_traj_idx][0])
                msg_raw.velocity.y = float(self.traj_vels[self.current_traj_idx][1])
                msg_raw.velocity.z = float(self.traj_vels[self.current_traj_idx][2])

                #Asignar Aceleración (qdd.txt)
                msg_raw.acceleration_or_force.x = float(self.traj_accs[self.current_traj_idx][0])
                msg_raw.acceleration_or_force.y = float(self.traj_accs[self.current_traj_idx][1])
                msg_raw.acceleration_or_force.z = float(self.traj_accs[self.current_traj_idx][2])
                
                self.raw_pub.publish(msg_raw)
                self.current_traj_idx += 1
            else:
                self.get_logger().info("Trayectoria Finalizada. Aterrizando...")
                self.mission_state = "LAND"

        # 4. ATERRIZAJE
        elif self.mission_state == "LAND":
            self.land_drone()
            self.mission_state = "DONE"
            
        elif self.mission_state == "DONE":
            pass # Fin del multirun

    # --- FUNCIONES AUXILIARES ---
    def publish_hover_setpoint(self, pos):
        msg = PoseStamped()
        
        # ¡ESTAS DOS LÍNEAS SON VITALES PARA QUE PX4 ACEPTE EL DATO!
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        self.local_pos_pub.publish(msg)

    def set_offboard_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self.set_mode_client.call_async(req)

    def arm_drone(self):
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)
        
    def land_drone(self):
        req = CommandTOL.Request()
        self.land_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = VincentVanDroneNode()
    
    try:
        rclpy.spin(node) # Mantiene el nodo vivo ejecutando los callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Ejecución interrumpida manualmente.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()