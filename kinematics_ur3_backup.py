import socket
import time
import math
import numpy as np
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R
import rtde_receive

# -----------------------------------------------------
# CONFIGURACIÓN ROBOT
# -----------------------------------------------------
ROBOT_IP = "192.168.0.4"
RTDE_PORT = 30004
SOCKET_PORT = 30002

# -----------------------------------------------------
# CARGAR CADENA IKPY (URDF)
# -----------------------------------------------------
URDF_PATH = "robot_models/ur3.urdf"
# Cargar URDF con versión de IKPy que no soporta argumentos extra
ur3_chain = Chain.from_urdf_file(URDF_PATH)

print(f"[IK] Cadena IKPy cargada. Total links: {len(ur3_chain.links)}")

# Descubrir índices de articulaciones reales del UR3
joint_indices = []
for idx, link in enumerate(ur3_chain.links):
    if link.joint_type == "revolute":
        joint_indices.append(idx)

print(f"[IK] Joints activos detectados: {joint_indices}")

joint_indices = [2, 3, 4, 5, 6, 7]  # articulaciones reales del UR3

print(f"[IK] Cadena IKPy cargada. DOF detectados: {len(joint_indices)}\n")

print("=== Links detectados ===")
for i, link in enumerate(ur3_chain.links):
    print(f"{i}: name={link.name}, joint_type={link.joint_type}")
print("")


# -----------------------------------------------------
# LÍMITES DE SEGURIDAD (en radianes)
# -----------------------------------------------------
SAFE_LIMITS = [
    (-2.8, 2.8),     # joint1
    (-2.0, -0.1),    # joint2 (evita singularidad codo arriba)
    (-2.8, 2.8),     # joint3
    (-3.0, 3.0),     # joint4
    (-2.0, 2.0),     # joint5
    (-3.0, 3.0)      # joint6
]

def clamp_joint_limits(q):
    """Recorta cada articulación a los límites de seguridad."""
    q_safe = []
    for i, angle in enumerate(q):
        low, high = SAFE_LIMITS[i]
        q_safe.append(max(low, min(high, angle)))
    return np.array(q_safe)


# -----------------------------------------------------
# RTDE Y SOCKET SCRIPT
# -----------------------------------------------------
print("[UR] Conectando RTDE (lectura de estados)...")
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

# -----------------------------------------
# 1. CONEXIÓN A UR3 — SOCKET (envío scripts)
# -----------------------------------------
print("[UR] Conectando Socket Script (envío de comandos)...")
try:
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  sock.connect((ROBOT_IP, SOCKET_PORT))
  print(f"[UR] Conectado a Robot(Socket) - Envio: {ROBOT_IP}:{SOCKET_PORT}")
except Exception as e:
  print(f"[UR] Error conectando a UR(Socket) - Envio: {e}")
  raise

# -----------------------------------------------------
# LECTURA DEL TCP REAL
# -----------------------------------------------------
def get_tcp_real():
    tcp = rtde_r.getActualTCPPose()
    return np.array([tcp[0], tcp[1], tcp[2]])

# -----------------------------------------------------
# ESPERAR A QUE EL ROBOT SE DETENGA
# -----------------------------------------------------
def wait_robot_stopped(threshold=0.001):
    while True:
        speed = np.linalg.norm(rtde_r.getActualTCPSpeed()[0:3])
        if speed < threshold:
            break
        time.sleep(0.02)


# -----------------------------------------------------
# POSICIÓN SEGURA DE ALINEACIÓN
# -----------------------------------------------------
SAFE_ALIGN_Q = [
    0.0,
    -math.pi/2,
    0.0,
    -math.pi/2,
    0.0,
    0.0
]
HOME_POINT_CMD = "movej([0, -1.57, 0, -1.57, 0, 0], a=1.0, v=0.5)\n"

def move_safe_align():
    print("\n[SAFE] Moviendo a ALIGN SAFE POSE...")
    # cmd = (
    #     f"movej([{','.join(map(str,SAFE_ALIGN_Q))}], a=0.5, v=0.2)\n"
    # )
    # sock.send(cmd.encode())
    sock.send(HOME_POINT_CMD.encode())
    wait_robot_stopped()
    print("[SAFE] Robot en pose segura.\n")


# -----------------------------------------------------
# MOVIMIENTO CARTESIANO SUAVE (trayectoria A → B)
# -----------------------------------------------------
def move_cartesian_smooth(target_xyz, steps=60):
    """
    Movimiento seguro:
    ✔ Interpolación lineal
    ✔ IKPy con orientación del TCP hacia abajo
    ✔ Límites de seguridad
    ✔ Envío suave moveJ
    """

    print("=== MOVIMIENTO SUAVE CON ORIENTACIÓN FIJA ===")

    p_start = get_tcp_real()
    p_end = np.array(target_xyz)

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
        sol = ur3_chain.inverse_kinematics(
            target_position=p.tolist(),
            target_orientation=R_fixed,
            orientation_mode="all"
        )

        q = sol[joint_indices]  # convertir IKPy → UR3 real
        q = clamp_joint_limits(q)

        # Enviar movimiento suave
        cmd = f"movej([{','.join(map(str,q))}], a=0.5, v=0.6)\n"
        sock.send(cmd.encode())

        time.sleep(0.05)

    wait_robot_stopped()

    # Evaluación final del movimiento
    tcp_final = get_tcp_real()
    err = tcp_final - p_end

    print("\n===== FIN MOVIMIENTO SUAVE =====")
    print("TCP REAL FINAL:", tcp_final)
    print("TARGET:", p_end)
    print(f"ERROR FINAL: dx={err[0]:.4f}, dy={err[1]:.4f}, dz={err[2]:.4f}")
    print("================================\n")


# =====================================================
# PROGRAMA PRINCIPAL
# =====================================================
if __name__ == "__main__":

    move_safe_align()

    print("\n=== CONTROL CARTESIANO MULTI-TARGET ===")
    print("Ingresa coordenadas X Y Z en metros (UR frame).")
    print("Ejemplo:   0.2 -0.1 0.5")
    print("Escribe 'exit' para salir.\n")

    while True:
        user = input("Target XYZ > ")

        if user.strip().lower() == "exit":
            break

        try:
            x, y, z = map(float, user.split())
        except:
            print("Formato inválido. Usa: x y z")
            continue

        target = np.array([x, y, z])
        print(f"\n→ Moviendo hacia: {target}")
        move_cartesian_smooth(target)

    print("\n[UR] Socket cerrado.")
    sock.close()

