import socket
import time
import math

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

def send_urscript(cmd, sock):
    """Envía texto URScript al robot."""
    sock.sendall(cmd.encode('utf-8'))


def connect_socket():
    """Abre socket hacia UR / URSim."""
    print(f"🟡 Conectando a {ROBOT_IP}:{SOCKET_PORT} ...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ROBOT_IP, SOCKET_PORT))

    print("🟢 Conectado por socket.")
    return sock


# -----------------------------------------------------
# MOVIMIENTO DE JOINTS
# -----------------------------------------------------

def moveJ(q, sock):
    """Envía movimiento moveJ en radianes."""
    cmd = f"movej({q}, a={ACCEL}, v={VEL})\n"
    send_urscript(cmd, sock)


def test_joint(joint_id, sock):
    print(f"\n===== Probando Joint {joint_id+1} =====")

    # Estado base
    # q = [0, 0, 0, 0, 0, 0]
    q= [0, -1.57, 0, -1.57, 0, 0]

    # +30 grados (0.52 rad)
    q[joint_id] = q[joint_id] + 0.52
    print(f"→ Joint {joint_id+1} = +30°")
    moveJ(q, sock)
    time.sleep(1.5)

    # -30 grados
    q[joint_id] = q[joint_id] -0.52
    print(f"→ Joint {joint_id+1} = -30°")
    moveJ(q, sock)
    time.sleep(1.5)

    # Volver a cero
    q[joint_id] = q[joint_id]
    print(f"→ Joint {joint_id+1} = 0°")
    moveJ(q, sock)
    time.sleep(1.5)


# -----------------------------------------------------
# EJECUCIÓN PRINCIPAL
# -----------------------------------------------------
if __name__ == "__main__":
    sock = connect_socket()

    print("\n🔵 Moviendo robot a [0,0,0,0,0,0] ...")
    moveJ([0, -1.57, 0, -1.57, 0, 0], sock)
    time.sleep(2)

    # Probar los 6 joints
    # for j in range(1):
    #     test_joint(j+1, sock)
    test_joint(5, sock)

    print("\n🟢 Prueba completada.")
    sock.close()
