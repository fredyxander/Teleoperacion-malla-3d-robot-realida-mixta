import asyncio
import websockets
import socket
import rtde_receive
import math

# -----------------------
# Configuración del robot
# -----------------------
ROBOT_IP = "192.168.0.4"  # URSim (o robot real si cambias la IP)
SOCKET_PORT = 30002         # Script interface
UR_PORT_RTDE = 30004        # Puerto RTDE

# -----------------------------------------
# 1. CONEXIÓN A UR3 — SOCKET (envío scripts)
# -----------------------------------------
try:
  r_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  r_socket.connect((ROBOT_IP, SOCKET_PORT))
  print(f"[UR] Conectado a Robot(Socket) - Envio: {ROBOT_IP}:{SOCKET_PORT}")
except Exception as e:
  print(f"[UR] Error conectando a UR(Socket) - Envio: {e}")
  raise

# -----------------------------------------
# 2. CONEXIÓN A UR3 — RTDE (lectura datos)
# -----------------------------------------
try:
  rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
  print(f"[RTDE] Conectado a Robot(RTDE) {ROBOT_IP}:{UR_PORT_RTDE}")
except Exception as e:
  print(f"[RTDE] Error conectando Robot(RTDE): {e}")
  raise

# -----------------------------------------
# LÍMITES DE SEGURIDAD
# -----------------------------------------
JOINT_LIMITS = {
  0: (math.radians(-340), math.radians(340)),
  1: (math.radians(-120), math.radians(-40)),
  2: (math.radians(-80), math.radians(80)),
  3: (math.radians(-300), math.radians(300)),
  4: (math.radians(-300), math.radians(300)),
  5: (math.radians(-200), math.radians(200)),
}

SAFE_MARGIN = math.radians(10)

HOME_POINT_CMD = "movej([0, -1.57, 0, -1.57, 0, 0], a=1.0, v=0.5)\n"

# -----------------------
# WebSocket
# -----------------------
connected_clients = set()
current_cmd = None


# 🔹 ENVÍA MENSAJE A TODOS LOS CLIENTES (Unity)
async def notify_clients(message: str):
  if connected_clients:
    await asyncio.gather(*[ws.send(message) for ws in connected_clients])


# 🔹 MANEJADOR DE CLIENTES UNITY
async def handle_client(websocket, path=None):
  global current_cmd

  client = websocket.remote_address
  print(f"[WS] Cliente conectado: {client}")

  connected_clients.add(websocket)

  # ---------------------------------------------------
  # Enviar mensaje al conectar (Unity mostrará "Conectado")
  # ---------------------------------------------------
  await websocket.send("Conexion establecida con servidor UR3")

  try:
    async for message in websocket:
      print(f"[WS] Mensaje recibido desde Unity: {message}")

      # Setear comando actual
      current_cmd = message

      # STOP inmediato (además del motion_loop)
      if message == "stop":
        try:
          r_socket.send(b"stopj(1.0)\n")
        except Exception as e:
          print("[UR] Error enviando stop:", e)

  except websockets.exceptions.ConnectionClosedOK:
    print(f"[WS] Cliente desconectado limpiamente: {client}")

  except Exception as e:
    print(f"[WS] Error cliente: {e}")

  finally:
    connected_clients.remove(websocket)
    print(f"[WS] Conexión finalizada: {client}")


# 🔹 LÓGICA PRINCIPAL DEL ROBOT (125 Hz)
async def motion_loop():
    global current_cmd

    # Valores para detectar quietud / llegada al objetivo
    STILL_THRESHOLD = 0.001      # diferencia mínima entre articulaciones
    STILL_CHECKS = 20            # número de muestras
    STILL_REQUIRED = 15          # cuántas deben ser estables para considerar "quieto"

    while True:
        try:
            joints = rtde_r.getActualQ()

            # -------------------------------------------------------
            # 🔹 Caso 1: Sin comando activo
            # -------------------------------------------------------
            if current_cmd is None or current_cmd == "stop":
                await asyncio.sleep(0.008)
                continue

            # -------------------------------------------------------
            # 🔹 Caso 2: Comando HOME
            # -------------------------------------------------------
            if current_cmd == "home":

                # 1. Enviar movimiento HOME
                r_socket.send(HOME_POINT_CMD.encode())
                print("[HOME] Enviando a pos HOME al robot...")

                # 2. Definir pose HOME exacta
                HOME_JOINTS = [0, -1.57, 0, -1.57, 0, 0]
                TOL = 0.02  # tolerancia en radianes (~1.1 grados)

                print("[HOME] Monitoreando llegada al HOME...")

                # 3. Esperar hasta que esté dentro del rango de tolerancia
                while True:
                    await asyncio.sleep(0.05)

                    joints = rtde_r.getActualQ()

                    # medir diferencia por articulación
                    diffs = [abs(j - h) for j, h in zip(joints, HOME_JOINTS)]
                    # print("diffs:", diffs)

                    if all(d < TOL for d in diffs):
                        print("[HOME] El robot llegó al HOME dentro de la tolerancia.")
                        await notify_clients("home_reached")
                        break

                current_cmd = None
                continue

            # -------------------------------------------------------
            # 🔹 Caso 3: Movimiento de articulaciones (base, hombro, codo, muñecas…)
            # -------------------------------------------------------
            # BASE IZQUIERDA
            if current_cmd == "base_izquierda":
                limit = JOINT_LIMITS[0][0]
                if joints[0] <= limit + SAFE_MARGIN:
                    print("[SAFETY] Límite J0 izquierda")
                    r_socket.send(b"stopj(1.0)\n")
                    current_cmd = None
                    await notify_clients("limit_reached")
                else:
                    r_socket.send(b"speedj([-0.5,0,0,0,0,0], 1.0, 0.1)\n")

            # BASE DERECHA
            elif current_cmd == "base_derecha":
                limit = JOINT_LIMITS[0][1]
                if joints[0] >= limit - SAFE_MARGIN:
                    print("[SAFETY] Límite J0 derecha")
                    r_socket.send(b"stopj(1.0)\n")
                    current_cmd = None
                    await notify_clients("limit_reached")
                else:
                    r_socket.send(b"speedj([0.5,0,0,0,0,0], 1.0, 0.1)\n")

            # -------------------------------------------------------
            # Aquí puedes seguir agregando hombro, codo, muñeca1, muñeca2, muñeca3
            # según la estructura original
            # -------------------------------------------------------
        except Exception as e:
            print("[UR] Error en motion_loop:", e)

        await asyncio.sleep(0.008)  # frecuencia ~125 Hz


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

    async with websockets.serve(handle_client, WS_SERVER_IP, port):
        await motion_loop( )# mantiene el bucle vivo

# Arranca el programa asíncrono.
# Lanza tanto el servidor WebSocket como el bucle de control del robot.
if __name__ == "__main__":
  asyncio.run(main())
