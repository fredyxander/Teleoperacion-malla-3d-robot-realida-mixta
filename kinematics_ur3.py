import socket
import time
import math
import numpy as np
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R
import rtde_receive

class UR3Controller:
    def __init__(self, ip="192.168.0.4"):
        self.ROBOT_IP = ip
        self.rtde_r = None
        self.socket = None
        self.RTDE_PORT = 30004
        self.SOCKET_PORT = 30002
        self.URDF_PATH = "robot_models/ur3_no_gripper.urdf"
        self.joint_indices = []

        print("[UR] Inicializando UR3Controller...")
        self.connect_rtde()
        self.connect_socket()
        self.load_urdf()

    # ----------------------------------------------------------
    # CONEXIÓN RTDE (lectura de estados del robot)
    # ----------------------------------------------------------
    def connect_rtde(self):
        try:
            print("[UR] Conectando RTDEReceive...")
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ROBOT_IP)
            print("[UR] RTDE conectado.")
        except Exception as e:
            print("[UR] Error conectando RTDE:", e)
            raise


    # ----------------------------------------------------------
    # CONEXIÓN SOCKET PARA COMANDOS
    # ----------------------------------------------------------
    def connect_socket(self):
        try:
            print(f"[UR] Conectando Socket Script {self.ROBOT_IP}:{self.SOCKET_PORT}...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.ROBOT_IP, self.SOCKET_PORT))
            print(f"[UR] Conectado a Robot(Socket) - Envio: {self.ROBOT_IP}:{self.SOCKET_PORT}")
        except Exception as e:
            print(f"[UR] Error conectando a UR(Socket) - Envio: {e}")
            raise

    # ----------------------------------------------------------
    # CARGAR MODELO IKPY (URDF)
    # ----------------------------------------------------------
    def load_urdf(self):
        print("[UR] Cargando UR3 URDF en IKPy...")

        self.ur3_chain = Chain.from_urdf_file(
            self.URDF_PATH,
            active_links_mask= [False, True, True, True, True, True, True, False]
        )
        print(f"[IK] Cadena IKPy cargada. Total links: {len(self.ur3_chain.links)}")


        # Descubrir índices de articulaciones reales del UR3
        for idx, link in enumerate(self.ur3_chain.links):
            if link.joint_type == "revolute":
                self.joint_indices.append(idx)

        print(f"[IK] Joints activos detectados: {self.joint_indices}")

        self.joint_indices = [1, 2, 3, 4, 5, 6]  # articulaciones reales del UR3

        print(f"[IK] Cadena IKPy cargada. DOF detectados: {len(self.joint_indices)}\n")

        print("=== Links detectados ===")
        for i, link in enumerate(self.ur3_chain.links):
            print(f"{i}: name={link.name}, joint_type={link.joint_type}")
        print("")


    # ----------------------------------------------------------
    # FUNCIÓN: obtener TCP real del UR3
    # ----------------------------------------------------------
    def get_tcp_real(self):
        tcp = self.rtde_r.getActualTCPPose()
        px, py, pz = tcp[0], tcp[1], tcp[2]
        rx, ry, rz = tcp[3], tcp[4], tcp[5]
        return np.array([px, py, pz]), np.array([rx, ry, rz])

    # -----------------------------------------------------
    # ESPERAR A QUE EL ROBOT SE DETENGA
    # -----------------------------------------------------
    def wait_robot_stopped(self,threshold=0.001):
        while True:
            speed = np.linalg.norm(self.rtde_r.getActualTCPSpeed()[0:3])
            if speed < threshold:
                break
            time.sleep(0.02)

    # -----------------------------------------------------
    # LÍMITES DE SEGURIDAD (en radianes)
    # -----------------------------------------------------
    def clamp_joint_limits(self, q):
        SAFE_LIMITS = [
            (-2.8, 2.8),     # joint1
            (-2.0, -0.1),    # joint2 (evita singularidad codo arriba)
            (-2.8, 2.8),     # joint3
            (-3.0, 3.0),     # joint4
            (-2.0, 2.0),     # joint5
            (-3.0, 3.0)      # joint6
        ]

        q_safe = []
        for i, angle in enumerate(q):
            low, high = SAFE_LIMITS[i]
            q_safe.append(max(low, min(high, angle)))

        return np.array(q_safe)


    # -----------------------------------------------------
    # POSICIÓN SEGURA DE ALINEACIÓN
    # -----------------------------------------------------
    def move_safe_align(self):
        HOME_POINT_CMD = "movej([0, -1.57, 0, -1.57, 0, 0], a=1.0, v=0.5)\n"
        print("\n[SAFE] Moviendo a Home POSE...")
        self.socket.send(HOME_POINT_CMD.encode())
        self.wait_robot_stopped()
        print("[SAFE] Robot en pose segura.\n")


    # ----------------------------------------------------------
    # FUNCIÓN: resolver IK y mover en trayectoria segura
    # (NO la activamos todavía)
    # MOVIMIENTO CARTESIANO SUAVE (trayectoria A → B)
    # -----------------------------------------------------
    def move_cartesian_smooth(self, target_xyz, steps=60):
        """
        Movimiento seguro:
        ✔ Interpolación lineal
        ✔ IKPy con orientación del TCP hacia abajo
        ✔ Límites de seguridad
        ✔ Envío suave moveJ
        """

        # Asegurar forma correcta (3,)
        target_position = np.array(target_xyz, dtype=float).reshape(3,)

        print("=== MOVIMIENTO SUAVE CON ORIENTACIÓN FIJA ===")

        p_start, _ = self.get_tcp_real()
        p_end = np.array(target_position)

        print(f"Posición inicial (TCP): {p_start}")
        print(f"Target deseado       : {p_end}")

        # Orientación fija del TCP apuntando hacia abajo
        R_fixed = R.from_euler("xyz", [math.pi, 0, 0]).as_matrix()

        # Generar trayectoria interpolada
        path = [
            p_start + (p_end - p_start) * (i / steps)
            for i in range(1, steps + 1)
        ]

        for i, p in enumerate(path):
            print(f"[{i+1}/{steps}] Resolviendo IK hacia {p}")

            # IK con orientación fija
            sol = self.ur3_chain.inverse_kinematics(
                target_position=p.tolist(),
                target_orientation=R_fixed,
                orientation_mode="all"
            )

            q = sol[self.joint_indices]  # convertir IKPy → UR3 real
            print(f"q: {q}")
            q = self.clamp_joint_limits(q)

            # Enviar movimiento suave
            cmd = f"movej([{','.join(map(str,q))}], a=0.5, v=0.6)\n"
            self.socket.send(cmd.encode())

            time.sleep(0.05)

        self.wait_robot_stopped()

        # Evaluación final del movimiento
        tcp_final_pos, tcp_final_rot = self.get_tcp_real()
        err = tcp_final_pos - p_end

        print("\n===== FIN MOVIMIENTO SUAVE =====")
        print("TCP REAL FINAL (pos):", tcp_final_pos)
        print("TCP REAL FINAL (rot):", tcp_final_rot)
        print("TARGET:", p_end)
        print(f"ERROR FINAL: dx={err[0]:.4f}, dy={err[1]:.4f}, dz={err[2]:.4f}")
        print("================================\n")

    # ----------------------------------------------------------
    # Cerrar conexiones
    # ----------------------------------------------------------
    def close(self):
        print("[UR] Cerrando conexiones...")
        if self.socket:
            self.socket.close()
        print("[UR] Conexiones cerradas.")
