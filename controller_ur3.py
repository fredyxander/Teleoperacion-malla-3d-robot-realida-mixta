import socket
import asyncio
import time
import math
import numpy as np
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R
import rtde_receive


class UR3ControllerIK:
    """
    Controlador completo UR3:
    - Carga URDF con IKPy
    - Conexión RTDE (lectura)
    - Conexión por Socket Script (envío)
    - IK cartesiano con orientación fija
    - Interpolador A → B suave
    """

    SAFE_LIMITS = [
        (-2.8, 2.8),     # J1
        (-2.0, -0.1),    # J2
        (-2.8, 2.8),     # J3
        (-3.0, 3.0),     # J4
        (-2.0, 2.0),     # J5
        (-3.0, 3.0)      # J6
    ]

    def __init__(self, robot_ip, urdf_path, rtde_port=30004, socket_port=30002):
        self.robot_ip = robot_ip
        self.rtde_port = rtde_port
        self.socket_port = socket_port
        self.urdf_path = urdf_path

        print("\n[IK] Cargando URDF...")
        self.chain = Chain.from_urdf_file(self.urdf_path)
        print(f"[IK] URDF cargado con {len(self.chain.links)} links.")

        # Índices reales del UR3 (confirmados)
        self.joint_indices = [2, 3, 4, 5, 6, 7]

        print("[IK] Joints reales:")
        for idx in self.joint_indices:
            print(f"  {idx}: {self.chain.links[idx].name}")

        print("\n[RTDE] Conectando...")
        self.rtde = rtde_receive.RTDEReceiveInterface(self.robot_ip)

        print("[SOCKET] Conectando Script...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.robot_ip, self.socket_port))
        print("[SOCKET] Conexión OK.\n")

    # ---------------------------------------------------------
    # LECTURAS
    # ---------------------------------------------------------
    def get_tcp_real(self):
        tcp = self.rtde.getActualTCPPose()
        return np.array(tcp[:3])

    def wait_robot_stopped(self, threshold=0.001):
        """Espera a que el robot deje de moverse."""
        while True:
            speed = np.linalg.norm(self.rtde.getActualTCPSpeed()[0:3])
            if speed < threshold:
                break
            time.sleep(0.02)

    async def async_wait_robot_stopped(self, threshold=0.001):
        """Versión asíncrona: no bloquea asyncio y permite streaming en vivo."""

        # Esperar a que el robot COMIENCE a moverse
        while True:
            speed = np.linalg.norm(self.rtde.getActualTCPSpeed()[0:3])
            if speed > threshold:
                break
            await asyncio.sleep(0.01)

        # Esperar a que el robot se DETENGA
        still_counter = 0
        required_frames = 5

        while True:
            speed = np.linalg.norm(self.rtde.getActualTCPSpeed()[0:3])

            if speed < threshold:
                still_counter += 1
            else:
                still_counter = 0

            if still_counter >= required_frames:
                break

            await asyncio.sleep(0.01)


    # ---------------------------------------------------------
    # ENVIAR COMANDOS
    # ---------------------------------------------------------
    def send_movej(self, q, a=1.0, v=1.0):
        cmd = f"movej([{','.join(map(str, q))}], a={a}, v={v})\n"
        self.sock.send(cmd.encode())

    def move_home(self, a=1.0, v=1.0):
        """
        Mueve el UR3 a la posición HOME:
        [0, -1.57, 0, -1.57, 0, 0]
        """
        home_q = [0, -1.57, 0, -1.57, 0, 0]
        print("\n[UR3] Moviendo a HOME...")

        self.send_movej(home_q, a=a, v=v)
        self.wait_robot_stopped()

        final_tcp = self.get_tcp_real()
        print("[UR3] HOME alcanzado. TCP:", final_tcp)

        return home_q

    async def move_home_async(self, a=1.0, v=1.0):
        home_q = [0, -1.57, 0, -1.57, 0, 0]

        print("\n[UR3] Moviendo a HOME...")

        self.send_movej(home_q, a=a, v=v)
        await self.async_wait_robot_stopped()

        final_tcp = self.get_tcp_real()
        print("[UR3] HOME alcanzado. TCP:", final_tcp)

        return home_q

    # ---------------------------------------------------------
    # UTILIDADES
    # ---------------------------------------------------------
    def clamp_joints(self, q):
        q_safe = []
        for i, angle in enumerate(q):
            low, high = self.SAFE_LIMITS[i]
            q_safe.append(max(low, min(high, angle)))
        return np.array(q_safe)

    # ---------------------------------------------------------
    # MOVIMIENTO CARTESIANO SUAVE
    # ---------------------------------------------------------
    def move_cartesian_smooth(self, target_xyz, steps=60):
        """
        Movimiento real:
        - Transformación URDF → URSim
        - IK fijo
        - Interpolación suave
        """

        print("\n=== MOVIMIENTO CARTESIANO (IKPy → UR3) ===")

        # 🔥 TRANSFORMACIÓN NECESARIA
        target_converted = np.array([
            -target_xyz[0],   # X invertido
            -target_xyz[1],   # Y invertido
            target_xyz[2]     # Z igual
        ])

        print("[TARGET] Original :", target_xyz)
        print("[TARGET] Convertido:", target_converted)

        p_start = self.get_tcp_real()
        p_end = target_converted

        print("[TCP] inicio:", p_start)
        print("[TCP] fin    :", p_end)

        # Orientación fija "mirando hacia abajo"
        R_fixed = R.from_euler("xyz", [math.pi, 0, 0]).as_matrix()

        # Trayectoria lineal
        path = [
            p_start + (p_end - p_start) * (i / steps)
            for i in range(1, steps + 1)
        ]

        global q_last
        for i, p in enumerate(path):
            print(f"[{i+1}/{steps}] IK → {p}")

            sol = self.chain.inverse_kinematics(
                target_position=p.tolist(),
                target_orientation=R_fixed,
                orientation_mode="all"
            )

            q_last = sol[self.joint_indices]
            q_last = self.clamp_joints(q_last)

        # Resultados finales
        final_tcp = self.get_tcp_real()
        err = final_tcp - p_end
        self.send_movej(q_last, a=1.0, v=1.0)
        # self.wait_robot_stopped()

        print("\n===== RESULTADO FINAL =====")
        print("TCP REAL :", final_tcp)
        print("TARGET   :", p_end)
        print("ERROR    :", err)
        print("===========================\n")
        return

    # ---------------------------------------------------------
    # CERRAR CONEXIONES
    # ---------------------------------------------------------
    def close(self):
        self.sock.close()
        print("[UR3] Socket cerrado.")
