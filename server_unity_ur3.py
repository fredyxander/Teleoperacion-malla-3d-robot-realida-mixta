import asyncio
import json
import numpy as np
import websockets
import time
import math
# Importar controlador UR3 con IK
from controller_ur3 import UR3ControllerIK

# =====================================================
# CONFIGURACIÓN
# =====================================================
ROBOT_IP = "192.168.0.4"
URDF_PATH = "robot_models/ur3.urdf"
WS_SERVER_IP = "192.168.0.3"
WS_PORT = 8765


# =====================================================
# ESTADO GLOBAL ESTRUCTURADO
# =====================================================
# Velocidad y aceleración seguras para pruebas
ACCEL = 1.0
VEL = 1.0
class CalibrationState:
    def __init__(self):
        self.unity_tcp_pos = None
        self.unity_tcp_rot = None
        self.offset_pos = None
        self.calibrated = False
        self.msg_type = None

STATE = CalibrationState()

# =====================================================
# CONTROLADOR UR3
# =====================================================
robot = UR3ControllerIK(robot_ip=ROBOT_IP, urdf_path=URDF_PATH)

# =====================================================
# LOGGING
# =====================================================
def log(msg):
    print(f"[PYTHON SERVER] {msg}")

# =====================================================
# CALIBRACIÓN UNITY → UR3
# =====================================================
# PROCESAR DIGITAL TCP Y CALIBRAR (Unity YA usa UR-frame)
def handle_digital_tcp_and_calibrate(data):
    STATE.unity_tcp_pos = np.array(data["position"], dtype=float)
    STATE.unity_tcp_rot = np.array(data["rotation"], dtype=float)

    log(f"[DIGITAL TCP UNITY] {STATE.unity_tcp_pos}")

    # Leer TCP real del UR3 desde IK Controller
    real_tcp_pos = robot.get_tcp_real()
    log(f"[REAL TCP (UR3)] {real_tcp_pos}")

    # Solo compensación POSICIÓN (rotación ya está correcta en Unity)
    STATE.offset_pos = real_tcp_pos - STATE.unity_tcp_pos
    STATE.calibrated = True

    log("======== CALIBRACIÓN COMPLETA ========")
    log(f"OFFSET POS = {STATE.offset_pos}")
    log("=======================================")

    return

# =====================================================
# TRANSFORMACIONES UNITY → UR3 FRAME
# =====================================================
# CONVERSIÓN POSICIONES TARGET UNITY → UR3 FRAME
def unity_to_ur3(pos_unity):
    """
    Convierte posición Unity → UR3 usando offset de calibración.
    Unity YA trabaja en UR-frame, solo requiere corrección.
    """
    p = np.array(pos_unity, dtype=float)

    # SISTEMA DE COORDENADAS:
    # Unity X+ → UR3 X-   (invertir)
    # Y se conserva
    # Z se conserva

    p_ur = np.array([
        -p[0],
        p[1],
        p[2]
    ])

    # 1. Aplicar rotación de calibración
    # p_rot = rot_offset.apply(pos_u)

    # 2. Aplicar offset de posición
    # Unity ya convierte → UR-frame → solo aplicar offset

    # Aplicar offset calibrado
    if STATE.offset_pos is not None:
        p_ur += STATE.offset_pos

    return p_ur

# =====================================================
# PROCESAR TARGET IK
# =====================================================
async def handle_target(websocket, data):
    """Procesa un target enviado desde Unity, convierte al marco UR3 y ejecuta movimiento IK."""
    if not STATE.calibrated:
        log("❌ No se puede procesar target: calibración NO lista.")
        return

    # 1. Leer target Unity
    target_unity = np.array(data["position"], dtype=float)
    log(f"[TARGET UNITY] {target_unity}")

    # 2. Convertir a coordenadas UR3
    target_ur = unity_to_ur3(target_unity)
    print("➡ TARGET Convertido (UR3) =", target_ur)

    # # 3. Ejecutar IK + movimiento suave del robot UR3
    log("🤖 Ejecutando movimiento IK...")
    robot.move_cartesian_smooth(target_ur)

    # 4. Obtener estado real del robot
    # Obtener joints al final
    q_end = robot.rtde.getActualQ()
    tcp_end = robot.get_tcp_real()

    log(f"[TCP FINAL] {tcp_end}")
    log(f"[JOINTS FINALES] {q_end}")

    # # 5. Enviar joints a Unity
    # log("Enviando q a Unity...")
    # msg = {
    #     "type": "joints_ur3_feedback",
    #     "joints": q_end
    # }
    # msg_json = json.dumps(msg)
    # await websocket.send(msg_json)
    # print("📤 Pose fija enviada a Unity:", msg_json)

# =====================================================
# PROCESAR COMANDO HOME
# =====================================================
async def handle_home(websocket):
    log("🏠 Movimiento HOME solicitado por Unity...")

    home_q = await robot.move_home_async()

    msg = {
        "type": "home_reached",
        "joints": home_q
    }
    msg_json = json.dumps(msg)
    await websocket.send(msg_json)
    print("📤 HOME alcanzado y enviado a Unity: ", msg_json)



async def stream_real_joints(websocket):
    while True:
        await asyncio.sleep(0.02)

        if not STATE.calibrated:
            continue

        q = robot.rtde.getActualQ()
        if(STATE.msg_type == "home"):
            q[4] += 1.57 * 2 # Invertir segundo joint para Unity

        msg = {
            "type": "joints_ur3_feedback",
            "joints": q
        }
        await websocket.send(json.dumps(msg))


# =====================================================
# PROCESAR MENSAJES RECIBIDOS
# =====================================================
async def process_message(websocket, data):
    STATE.msg_type = data.get("type", None)

    if STATE.msg_type == "digital_tcp":
        handle_digital_tcp_and_calibrate(data)

    elif STATE.msg_type == "ik_request":
        await handle_target(websocket, data)

    elif STATE.msg_type == "home":
        await handle_home(websocket)

    else:
        STATE.msg_type = None
        log(f"⚠ Mensaje desconocido: {data}")

# =====================================================
# HANDLER DE CLIENTE UNITY
# =====================================================
async def client_handler(websocket, path=None):
    log("Unity conectado. Esperando mensajes...")
    # Iniciar stream en paralelo
    asyncio.create_task(stream_real_joints(websocket))

    try:
        async for message in websocket:
            log(f"[RX] {message}")
            try:
                data = json.loads(message)
                await process_message(websocket, data)
            # except json.JSONDecodeError:
            #     log("⚠ ERROR: mensaje no es JSON válido.")
            except Exception as e:
                log(f"❌ Error procesando mensaje: {e}")

    except websockets.exceptions.ConnectionClosedOK:
        log("Unity desconectado.")

    except Exception as e:
        log(f"❌ Error inesperado: {e}")


# =====================================================
# SERVIDOR PRINCIPAL
# =====================================================
async def main():
    print("=====================================")
    print("🟢 Servidor WebSocket para Quest listo")
    print(f"  Conectarse vía: ws://{WS_SERVER_IP}:{WS_PORT}")
    print("=====================================")

    async with websockets.serve(client_handler, WS_SERVER_IP, WS_PORT):
        log("Servidor listo. Esperando conexiones...")
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
