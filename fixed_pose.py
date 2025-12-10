import asyncio
import socket
import websockets
import time
import json
import rtde_receive
import numpy as np

# -----------------------------------------------------
# CONFIGURACIÓN
# -----------------------------------------------------
ROBOT_IP = "192.168.0.4"
SOCKET_PORT = 30002

# Velocidad y aceleración seguras para pruebas
ACCEL = 1.0
VEL = 1.0

# -----------------------------------------------------
# FUNCIONES DE COMUNICACIÓN
# -----------------------------------------------------
print("[UR] Conectando RTDE (lectura de estados)...")
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

def connect_socket():
    """Abre socket hacia UR / URSim."""
    print(f"🟡 Conectando a {ROBOT_IP}:{SOCKET_PORT} ...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ROBOT_IP, SOCKET_PORT))

    print("🟢 Conectado por socket.")
    return sock

def send_urscript(cmd, sock):
    """Envía texto URScript al robot."""
    sock.sendall(cmd.encode('utf-8'))

# -----------------------------------------------------
# LOGGING
# -----------------------------------------------------
def log(msg):
    print(f"[PYTHON SERVER] {msg}")

# -----------------------------------------------------
# MOVIMIENTO DE JOINTS
# -----------------------------------------------------
def moveJ(q, sock):
    """Envía movimiento moveJ en radianes."""
    cmd = f"movej({q}, a={ACCEL}, v={VEL})\n"
    send_urscript(cmd, sock)

def send_urscript_move(q,sock):
    # Pose fija (en radianes)
    # Debes ajustar estos valores a una pose válida del UR3
    moveJ(q, sock)
    time.sleep(1.5)

# -----------------------------------------------------
# LECTURA DEL TCP REAL
# -----------------------------------------------------
def get_tcp_real():
    tcp = rtde_r.getActualTCPPose()
    return np.array([tcp[0], tcp[1], tcp[2]])

def get_tcp_q():
    joints = rtde_r.getActualQ()
    return joints

# -------------------------------------------------------------
# HANDLER DE CLIENTE UNITY fixed pose
# -------------------------------------------------------------
async def client_handler(websocket, path=None):
    log("Unity conectado.")

    sock = connect_socket()

    home = [0,  -1.57, 0, -1.57, 0, 0]
    log("\n🔵 Moviendo robot a home")
    moveJ(home, sock)
    time.sleep(2)

    # Obtener joints justo antes
    pos_start = get_tcp_real()
    q_start = rtde_r.getActualQ()
    print(f"🔵 TCP al INICIO: {pos_start}")
    print(f"🔵 Joints al INICIO: {q_start}")
    time.sleep(2)

    # Posicion fija
    deg1=45 * 3.1416 / 180  # Convertir grados a radianes
    deg2=90 * 3.1416 / 180  # Convertir grados a radianes
    q_fixed = [
        home[0] - deg1,  # Joint 1->  45
        home[1] - deg2,  # Joint 1->  45
        home[2] + deg1,  # Joint 3-> -90
        home[3] - deg1,  # Joint 4-> -90
        home[4] + deg2,  # Joint 5-> -90
        home[5] + deg1   # Joint 6->  90
    ]

    log(f"\n🔵 Moviendo robot a pose fija: {q_fixed}")
    send_urscript_move(q_fixed, sock) # Radianes a ur3

    log("\n🟢 Prueba ur completada.")

    # Esperar a que termine el movimiento
    # Estrategia: monitorear velocidad TCP
    while True:
        speed = abs(rtde_r.getActualTCPForce()[0])  # Fuerza → muy baja cuando termina
        joints_speed = sum(abs(x) for x in rtde_r.getActualQd())  # velocidad angular

        if joints_speed < 0.01:   # umbral bajo = robot detenido
            break

        time.sleep(0.05)

    # Obtener joints al final
    q_end = get_tcp_q()
    pos_end = get_tcp_real()
    print(f"🟢 TCP al FINAL: {pos_end}")
    print(f"🟢 Joints al FINAL: {q_end}")
    sock.close()

    log("Enviando q_fixed a Unity...")
    msg = {
        "type": "joint_fixed_pose",
        # "joints": q_fixed
        "joints": q_fixed
    }
    msg_json = json.dumps(msg)
    await websocket.send(msg_json)
    print("📤 Pose fija enviada a Unity:", msg_json)

    # Mantener conexión
    while True:
        try:
            data = await websocket.recv()
            print("Mensaje recibido de Unity:", data)
        except websockets.ConnectionClosed:
            log("❌ Unity desconectado")
            break

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