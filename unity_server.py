import asyncio
import websockets
import json
import numpy as np
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R

# Importar el archivo de cinemática (aún sin usar)
# from kinematics_ur3 import move_cartesian_smooth, get_tcp_real
# Más adelante activaremos esto cuando tengamos mapeo Unity→UR3
# De momento solo los dejamos importados pero comentados

# =========================================
# VARIABLES GLOBALES
# =========================================
unity_base_pos = None
unity_base_rot = None

unity_tcp_pos = None
unity_tcp_rot = None

ursim_tcp_pos = None
ursim_tcp_rot = None

calibration_ready = False

rot_offset = None
pos_offset = None

# -------------------------------------------------------------
# LOGGING
# -------------------------------------------------------------
def log(msg):
    print(f"[UNITY SERVER] {msg}")


# =========================================
# RECIBIR BASE POSE
# =========================================
def handle_base_pose(data):
    global unity_base_pos, unity_base_rot

    p = data["position"]
    q = data["rotation"]

    unity_base_pos = np.array(p)
    unity_base_rot = np.array(q)

    log(f"Unity BASE Pose → pos={p}, rot={q}")


# =========================================
# RECIBIR DIGITAL TCP
# =========================================
def handle_digital_tcp(data):
    global unity_tcp_pos, unity_tcp_rot

    p = data["position"]
    q = data["rotation"]

    unity_tcp_pos = np.array(p)
    unity_tcp_rot = np.array(q)

    log(f"Unity DIGITAL TCP → pos={p}, rot={q}")

# =========================================
# FUNCIÓN: intentar calibrar si es posible
# =========================================
def attempt_calibration():
    global calibration_ready, pos_offset, rot_offset

    if unity_tcp_pos is None or unity_tcp_rot is None:
        return False

    if ursim_tcp_pos is None or ursim_tcp_rot is None:
        return False

    # --- Rotación ---
    unity_R = R.from_quat(unity_tcp_rot)
    real_R = R.from_rotvec(ursim_tcp_rot)  # URSim da rotvec

    rot_offset = real_R * unity_R.inv()

    # --- Posición ---
    pos_offset = ursim_tcp_pos - rot_offset.apply(unity_tcp_pos)

    calibration_ready = True

    log("\n======= 🔧 CALIBRACIÓN COMPLETA =======")
    log(f"Rot Offset (quat) = {rot_offset.as_quat().tolist()}")
    log(f"Pos Offset (m)    = {pos_offset}")
    log("=====================================\n")

    return True

# =========================================
# PROCESAR MENSAJE JSON DESDE UNITY
# =========================================
async def process_message(websocket, data):
    global calibration_ready

    msg_type = data.get("type")

    # ---------------------------
    # Mensaje base_pose
    # ---------------------------
    if msg_type == "base_pose":
        handle_base_pose(data)
        return

    # ---------------------------
    # Mensaje digital_tcp
    # ---------------------------
    if msg_type == "digital_tcp":
        handle_digital_tcp(data)

        if attempt_calibration():
            log("💡 Calibración lista. Unity ↔ UR3 alineados.")
        else:
            log("Esperando datos reales del robot para calibrar…")

        return

    # ---------------------------
    # Mensajes desconocidos
    # ---------------------------
    log(f"Mensaje desconocido: {data}")


# =========================================
# SERVIDOR WEBSOCKET
# =========================================
async def client_handler(websocket,  path=None):
    log("Unity conectado.")

    try:
        async for message in websocket:
            log(f"[RX] {message}")

            try:
                data = json.loads(message)
                await process_message(websocket, data)

            except json.JSONDecodeError:
                log("⚠ Mensaje NO es JSON válido.")

    except websockets.exceptions.ConnectionClosedOK:
        log("Unity desconectado.")

    finally:
        log("Conexión finalizada con Unity.")


# 🔹 LÓGICA PRINCIPAL DEL ROBOT (125 Hz)
async def motion_loop():
    global current_cmd


# Cada vez que un cliente se conecta, llama a handle_client.
# Mientras tanto, arranca el bucle motion_loop que sigue mandando comandos a UR según el estado.
# Servidor WebSocket + motion loop
async def main():
    WS_SERVER_IP = "192.168.0.3"
    port = 8765

    print("=====================================")
    print("🟢 Servidor WebSocket corriendo para Quest")
    print(f"   url para meta quest: ws://{WS_SERVER_IP}:{port}")
    print("=====================================")

    async with websockets.serve(client_handler, WS_SERVER_IP, port):
        log("Servidor listo. Esperando conexiones...")
        # await motion_loop( )# mantiene el bucle vivo
        await asyncio.Future()  # keep alive forever

# Arranca el programa asíncrono.
# Lanza tanto el servidor WebSocket como el bucle de control del robot.
if __name__ == "__main__":
  asyncio.run(main())