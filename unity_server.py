import asyncio
import websockets
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

# Importar controlador del UR3
from kinematics_ur3 import UR3Controller

# ------------------------------------------------------
# INICIALIZAR CONTROLADOR DEL ROBOT
# ------------------------------------------------------
ROBOT_IP = "192.168.0.4"
ur3 = UR3Controller(ROBOT_IP)

# Variables globales
unity_base_pos = None
unity_base_rot = None

unity_tcp_pos = None
unity_tcp_rot = None

pos_offset = None
rot_offset = None
calibration_ready = False


# -------------------------------------------------------------
# 🔵 CONVERSIÓN POSICIONES TARGET UNITY → UR3 FRAME
# -------------------------------------------------------------
def unity_to_ur3(pos_u):
    global rot_offset, pos_offset, calibration_ready

    if not calibration_ready:
        print("❌ ERROR: la calibración no está lista.")
        return None

    pos_u = np.array(pos_u, dtype=float)

    # # 1. Aplicar rotación de calibración
    # p_rot = rot_offset.apply(pos_u)

    # 2. Aplicar offset de posición
    # Unity ya convierte → UR-frame → solo aplicar offset
    p_corr  = pos_u + pos_offset

    return p_corr


# -------------------------------------------------------------
# LOGGING
# -------------------------------------------------------------
def log(msg):
    print(f"[UNITY SERVER] {msg}")


# -------------------------------------------------------------
# PROCESAR POSE DE LA BASE
# -------------------------------------------------------------
def handle_base_pose(data):
    global unity_base_pos, unity_base_rot

    unity_base_pos = np.array(data["position"])
    unity_base_rot = np.array(data["rotation"])

    log(f"BASE Pose (Unity) = {unity_base_pos}, rot={unity_base_rot}")


# -------------------------------------------------------------
# PROCESAR DIGITAL TCP Y CALIBRAR (Unity YA usa UR-frame)
# -------------------------------------------------------------
def handle_digital_tcp_and_calibrate(data):
    global unity_tcp_pos, unity_tcp_rot
    global pos_offset, rot_offset, calibration_ready

    unity_tcp_pos = np.array(data["position"])
    unity_tcp_rot = np.array(data["rotation"])

    log(f"DIGITAL TCP (Unity) = {unity_tcp_pos}")

    # -----------------------------
    # Obtener TCP real del UR3
    # -----------------------------
    real_pos, real_rot = ur3.get_tcp_real()
    log(f"REAL TCP (UR3) = {real_pos}")

    # --------------------------------------------
    # CALIBRACIÓN: Solo traslación, sin rotación
    # (Unity ya está enviando UR-frame)
    # --------------------------------------------
    # unity_R = R.from_quat(unity_tcp_rot)
    # real_R = R.from_rotvec(real_rot)   # UR usa rotvec (Rx, Ry, Rz)
    # rot_offset = real_R * unity_R.inv()

    pos_offset = real_pos - unity_tcp_pos

    calibration_ready = True

    log("======= CALIBRACIÓN COMPLETA =======")
    log(f"pos_offset = {pos_offset}")
    log("=====================================")

    return

async def handle_target(websocket, data):
    """Procesa un target enviado desde Unity, convierte al marco UR3 y ejecuta movimiento IK."""

    if not calibration_ready:
        print("❌ No se puede procesar target: calibración NO lista.")
        return

    # -----------------------------
    # 1) Leer target desde Unity
    # -----------------------------
    target_unity = np.array(data["position"], dtype=float)
    print("\n🎯 TARGET (Unity) =", target_unity)

    # -----------------------------
    # 2) Convertir a coordenadas UR3
    # -----------------------------
    target_ur = unity_to_ur3(target_unity)
    # target_ur[0] = -target_ur[0]  # Invertir X para UR3
    # target_ur[1] = -target_ur[1]  # Invertir Y para UR3
    target_ur = np.array(target_ur, dtype=float).reshape(3,)
    print("➡ TARGET Convertido (UR3) =", target_ur)

    # -----------------------------
    # 3) Mover robot usando IK suave
    # -----------------------------
    print("\n🤖 Ejecutando movimiento IK...")
    ur3.move_cartesian_smooth(target_ur)

    # -----------------------------
    # 4) Leer estado real del robot
    # -----------------------------
    tcp_real, tcp_rot = ur3.get_tcp_real()
    joints_real = ur3.rtde_r.getActualQ()
    joints_deg = np.degrees(joints_real).tolist()
    # joints_real[0] = -joints_real[0]  # Invertir J1 para UR3
    # joints_real[1] = -joints_real[1]   # Invertir J2 para UR3
    # joints_real[2] = joints_real[2]
    # joints_real[3] = joints_real[3] * 1.9  # Invertir J4 para UR3
    # joints_real[4] = -joints_real[4] # Ajuste J5 para UR3
    # joints_real[5] = -joints_real[5]    # Invertir J6 para UR3
    print("📍 TCP REAL después del movimiento =", tcp_real)

    # -----------------------------
    # 5) Calcular error
    # -----------------------------
    err = tcp_real - target_ur
    print("📏 Error final =", err)

    # -----------------------------
    # 5) Enviar estado real a Unity
    # -----------------------------
    msg = {
        "type": "robot_state",
        "tcp_pos": tcp_real.tolist(),
        "tcp_rotvec": tcp_rot.tolist(),
        "joints": list(joints_real)
    }

    msg_json = json.dumps(msg)
    await websocket.send(msg_json)
    print("Joins en grados =", joints_deg)
    print("📤 Estado real enviado a Unity:", msg_json)

    print("--------------------------------------------------\n")

# -------------------------------------------------------------
# PROCESAR MENSAJES UNITY
# -------------------------------------------------------------
async def process_message(websocket, data):
    msg_type = data.get("type")

    if msg_type == "base_pose":
        handle_base_pose(data)
        return

    if msg_type == "digital_tcp":
        handle_digital_tcp_and_calibrate(data)
        return

    if msg_type == "ik_request":
        await handle_target(websocket, data)
        return

    log("⚠ Mensaje desconocido recibido.")


# -------------------------------------------------------------
# HANDLER DE CLIENTE UNITY
# -------------------------------------------------------------
async def client_handler(websocket, path=None):
    log("Unity conectado.")

    try:
        async for message in websocket:
            log(f"[RX] {message}")

            try:
                data = json.loads(message)
                await process_message(websocket, data)

            except json.JSONDecodeError:
                log("⚠ ERROR: mensaje no es JSON válido.")

    except websockets.exceptions.ConnectionClosedOK:
        log("Unity desconectado.")

    finally:
        log("Conexión finalizada con Unity.")


# -------------------------------------------------------------
# SERVIDOR PRINCIPAL
# -------------------------------------------------------------
async def main():
    WS_SERVER_IP = "192.168.0.3"
    PORT = 8765

    print("=====================================")
    print("🟢 Servidor WebSocket para Quest listo")
    print(f"  Conectarse vía: ws://{WS_SERVER_IP}:{PORT}")
    print("=====================================")

    async with websockets.serve(client_handler, WS_SERVER_IP, PORT):
        log("Servidor listo. Esperando conexiones...")
        await asyncio.Future()  # Mantener vivo


if __name__ == "__main__":
    asyncio.run(main())
