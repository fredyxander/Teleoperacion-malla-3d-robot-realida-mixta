import asyncio
import websockets
import socket
import rtde_receive
import math
import json
import numpy as np
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R

# ============================================================
#   SECCIÓN IKPy — Cargar modelo UR3 (URDF)
# ============================================================
last_unity_tcp = None
offset = [0.0, 0.0, 0.0]
offset_valid = False


URDF_PATH = "robot_models/ur3.urdf"   # <- pon tu URDF aquí

try:
    ur3_chain = Chain.from_urdf_file(URDF_PATH)
    # print("DOF from IKPy chain:", ur3_chain.active_links_mask)
    # print("Number of joints:", len(ur3_chain.links))
    print("[IKPY] UR3 URDF cargado correctamente.")
except Exception as e:
    print("[IKPY] ERROR cargando ur3.urdf:", e)
    raise

def deg_to_rad(q):
    return [math.radians(a) for a in q]

def quat_to_matrix(x, y, z, w):
    q = np.array([w, x, y, z])
    n = np.dot(q, q)
    if n < 1e-8:
        return np.eye(3)

    q *= np.sqrt(2.0 / n)
    q = np.outer(q, q)

    return np.array([
        [1 - q[2,2] - q[3,3],     q[1,2] - q[3,0],     q[1,3] + q[2,0]],
        [    q[1,2] + q[3,0], 1 - q[1,1] - q[3,3],     q[2,3] - q[1,0]],
        [    q[1,3] - q[2,0],     q[2,3] + q[1,0], 1 - q[1,1] - q[2,2]]
    ])

def solve_ik(px, py, pz):
    import numpy as np

    # Estado actual real del UR3 (para guiar a IKPy)
    current_q = rtde_r.getActualQ()  # radianes [q1..q6]

    # IKPy necesita 10 joints
    initial_guess = np.array([0.0] + list(current_q) + [0.0, 0.0, 0.0])

    sol = np.array(
        ur3_chain.inverse_kinematics(
            target_position=[px, py, pz],
            initial_position=initial_guess
        )
    )

    print("IKPy raw solution:", sol, "size:", len(sol))

    # Extraer articulaciones reales → joints 1..6
    sol6 = sol[1:7]

    # Normalizar a rango [-π, π]
    sol6 = (sol6 + np.pi) % (2*np.pi) - np.pi

    print("IK normalized:", sol6)

    # Retornar en RADIANES para moveJ
    return sol6.tolist()


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
  rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP) #leer estados
  print(f"[RTDE] Conectado a Robot(RTDE) {ROBOT_IP}:{UR_PORT_RTDE}")
  tcp_pose = rtde_r.getActualTCPPose()
  rot_real = R.from_rotvec([tcp_pose[3], tcp_pose[4], tcp_pose[5]]).as_quat()
  # formato [x,y,z,w]

  print("\n======= REAL TCP REPORT (from URSim) =========")
  print(f"URSim TCP (UR frame) = ({tcp_pose[0]:.4f}, {tcp_pose[1]:.4f}, {tcp_pose[2]:.4f})")
  print("===============================================\n")

  # Y si Unity ya envió el suyo:
  if last_unity_tcp is not None:
      dx = last_unity_tcp[0] - tcp_pose[0]
      dy = last_unity_tcp[1] - tcp_pose[1]
      dz = last_unity_tcp[2] - tcp_pose[2]
      print(f"Δ (UnityTCP - URSimTCP) = ({dx:.4f}, {dy:.4f}, {dz:.4f})")
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

  global offset
  global rot_offset
  global calibration_ready
  calibration_ready = False

  try:
    async for message in websocket:
      print(f"[WS] Mensaje recibido desde Unity: {message}")

      # JSON — IK Request
      try:
        data = json.loads(message)

        # ======================================
        # 🔵 IK REQUEST DESDE UNITY
        # ======================================
        if data.get("type") == "digital_tcp":
          # ---------------------------
          # 1. Leer TCP digital de Unity
          # ---------------------------
          ux, uy, uz = data["position"]
          uq = data["rotation"]   # quaternion [x,y,z,w] en marco UR (Unity)

          print("======= DIGITAL TCP REPORT (from Unity) =======")
          print(f"Unity TCP (UR frame) = ({ux:.4f}, {uy:.4f}, {uz:.4f})")
          print(f"Unity Rot (quat)     = {uq}")

          # ---------------------------
          # 2. Leer TCP real URSim
          # ---------------------------
          tcp = rtde_r.getActualTCPPose()
          rx, ry, rz = tcp[0], tcp[1], tcp[2]
          rrx, rry, rrz = tcp[3], tcp[4], tcp[5]  # rotvec

          from scipy.spatial.transform import Rotation as R
          real_rot_q = R.from_rotvec([rrx, rry, rrz]).as_quat()

          print("\n======= REAL TCP REPORT (from URSim) =========")
          print(f"URSim TCP (UR frame) = ({rx:.4f}, {ry:.4f}, {rz:.4f})")
          print(f"URSim Rot (quat)     = {real_rot_q.tolist()}")

          # ---------------------------
          # 3. Calcular offset de posición
          # ---------------------------
          # 2. ROTALES (ya lo tienes)
          unity_rot = R.from_quat(uq)
          real_rot  = R.from_quat(real_rot_q)
          rot_offset = real_rot * unity_rot.inv()

          # 3. POSICIÓN CORRECTA (NUEVO)
          unity_pos = np.array([ux, uy, uz])
          real_pos  = np.array([rx, ry, rz])

          offset = real_pos - rot_offset.apply(unity_pos)   # ✅ ESTA es la clave

          print("\n======= CALIBRACIÓN POSICIÓN =======")
          print(f"Offset = {offset}")
          print("======= CALIBRACIÓN ROTACIÓN =======")
          print(f"Rotation Offset (quat) = {rot_offset.as_quat().tolist()}")

          calibration_ready = True
          print("=== CALIBRACIÓN COMPLETA ===")
          print("====================================\n")
          continue  # ← importante


        if data.get("type") == "ik_request":
          if not calibration_ready:
            print("ERROR: Calibración no realizada todavía.")
            continue

          # 1. Leer posición del target (Unity)
          px, py, pz = data["position"]
          p = np.array([px, py, pz])

          print(f"📌 Target recibido (Unity / sin offset): {p}")

          # 2. Aplicar corrección de rotación
          try:
            p_rot = rot_offset.apply(p)
          except Exception as e:
            print("[IK] Error aplicando rot_offset:", e)
            continue

          # 3. Aplicar corrección de traslación
          p_corr = p_rot + offset
          print(f"📌 Target corregido (UR frame): {p_corr}")

          # 4. Resolver IK con IKPy (retorna radianes)
          try:
            q_sol = solve_ik(p_corr[0], p_corr[1], p_corr[2])  # radianes
          except Exception as e:
            print("[IK] Error en solve_ik:", e)
            # Avisar a Unity del fallo:
            error_msg = {
              "type": "ik_error",
              "reason": str(e)
            }
            await websocket.send(json.dumps(error_msg))
            continue

          print(f"[IK] Solución final (rads): {q_sol}")

          # # 5. Enviar comando moveJ (socket 30002)
          # try:
          #   print("[IK] Enviando moveJ al UR3...")
          #   cmd = f"movej([{q_sol[0]},{q_sol[1]},{q_sol[2]},{q_sol[3]},{q_sol[4]},{q_sol[5]}], a=0.5, v=0.2)\n"
          #   r_socket.send(cmd.encode())
          #   print(f"[UR] moveJ enviado → {q_sol}")

          #   # # # 4. Enviar solución a Unity
          #   # reply = {
          #   #   "type": "ik_solution",
          #   #   "q": q_sol.tolist()
          #   # }
          #   # await websocket.send(json.dumps(reply))
          # except Exception as e:
          #   print("[UR] Error enviando moveJ:", e)

          # 5. Enviar comando moveJ (socket 30002)
          try:
              print("[IK] Enviando moveJ al UR3...")

              cmd = (
                  f"movej([{q_sol[0]},{q_sol[1]},{q_sol[2]},"
                  f"{q_sol[3]},{q_sol[4]},{q_sol[5]}], a=0.5, v=0.2)\n"
              )
              r_socket.send(cmd.encode())
              print(f"[UR] moveJ enviado → {q_sol}")

              # ------------------------------------------------------------
              # 🔍 6. ESPERAR HASTA QUE EL ROBOT FINALICE EL MOVIMIENTO
              # ------------------------------------------------------------
              import time

              TOLERANCE = 0.005     # 5 mm precisión
              STILL_SPEED = 0.01    # velocidad mínima (1 cm/s)
              MAX_TIME = 12         # timeout

              start_time = time.time()

              while True:
                  await asyncio.sleep(0.05)

                  # Posición y velocidad reales
                  tcp_now = np.array(rtde_r.getActualTCPPose()[0:3])
                  tcp_speed = np.array(rtde_r.getActualTCPSpeed()[0:3])
                  speed_norm = np.linalg.norm(tcp_speed)

                  # Error vs target corregido
                  error_vec = p_corr - tcp_now
                  error_norm = np.linalg.norm(error_vec)

                  print(f"[Wait] Error={error_norm:.4f} m | Vel={speed_norm:.4f} m/s")

                  # 1) Condición de llegada
                  if error_norm < TOLERANCE:
                      print("✅ Robot llegó al target (por error < tolerancia).")
                      break

                  # 2) Robot casi detenido
                  if speed_norm < STILL_SPEED:
                      print("🟡 Robot detenido — fin del movimiento.")
                      break

                  # 3) Timeout
                  if time.time() - start_time > MAX_TIME:
                      print("⚠️ Timeout — movimiento no finalizó.")
                      break

              # ------------------------------------------------------------
              # 🔍 7. REPORTE FINAL DE PRECISIÓN
              # ------------------------------------------------------------
              tcp_final = np.array(rtde_r.getActualTCPPose()[0:3])
              final_error = p_corr - tcp_final

              print("\n===== RESULTADO FINAL DEL MOVIMIENTO =====")
              print(f"TCP REAL FINAL = ({tcp_final[0]:.4f}, {tcp_final[1]:.4f}, {tcp_final[2]:.4f})")
              print(f"TARGET CORREGIDO = ({p_corr[0]:.4f}, {p_corr[1]:.4f}, {p_corr[2]:.4f})")
              print(
                  f"ERROR FINAL = (dx={final_error[0]:.4f}, "
                  f"dy={final_error[1]:.4f}, dz={final_error[2]:.4f})"
              )
              print("==========================================\n")

          except Exception as e:
              print("[UR] Error enviando moveJ o verificando TCP:", e)
          continue  # ← importante

      except:
        pass  # el mensaje no era JSON

      # -------------------------
      # Comandos antiguos
      # -------------------------
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
            pose = rtde_r.getActualTCPPose()  # [x,y,z, rx,ry,rz]

            # await r_socket.send(json.dumps({
            #     "type": "robot_tcp",
            #     "tcp_pos": pose[:3],
            #     "tcp_rot": pose[3:]
            # }))

# home
# [URSim TCP] pos = [-0.0004996551515492758, -0.2231500000936809, 0.6939497468862921]
# [URSim Joint] q(rad) = [0.0, -1.5700000000000003, 0.0, -1.5700000000000003, 0.0, -8.881784197001252e-16]

            # print(f"[URSim TCP] pos = {pose[:3]}")
            # print(f"[URSim Joint] q(rad) = {joints}")
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
