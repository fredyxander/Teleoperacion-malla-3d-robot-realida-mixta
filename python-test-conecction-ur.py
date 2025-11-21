import socket
import rtde_receive

# -----------------------
# Configuración del robot
# -----------------------
UR_IP = "192.168.0.122"   # Cambia por la IP de tu UR real o de URSim
UR_PORT_SOCKET = 30002   # Puerto para enviar programas/scripts al robot
UR_PORT_RTDE = 30004     # Puerto de RTDE

def connect_socket(ip, port):
    """Conexión básica vía socket para enviar scripts"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ip, port))
        print(f"[SOCKET] Conectado a {ip}:{port}")
        return s
    except Exception as e:
        print(f"[SOCKET] Error conectando: {e}")
        return None

def connect_rtde(ip):
    """Conexión vía RTDE para recibir estado"""
    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(ip)
        print(f"[RTDE] Conectado a {ip}")
        return rtde_r
    except Exception as e:
        print(f"[RTDE] Error conectando: {e}")
        return None

if __name__ == "__main__":
    # Conexiones
    socket_conn = connect_socket(UR_IP, UR_PORT_SOCKET)
    rtde_conn = connect_rtde(UR_IP)

    if socket_conn and rtde_conn:
        try:
            # Enviar un comando simple por socket para enviar al home el robot
            cmd = "def test_prog():\n movej([0, -1.57, 0, -1.57, 0, 0], a=1.0, v=0.5)\nend\n"
            socket_conn.send(cmd.encode('utf-8'))
            print("[SOCKET] Programa enviado al robot.")

            # Leer estado actual con RTDE
            actual_q = rtde_conn.getActualQ()
            tcp_pose = rtde_conn.getActualTCPPose()
            print(f"[RTDE] Posición articular: {actual_q}")
            print(f"[RTDE] Pose TCP: {tcp_pose}")

        finally:
            socket_conn.close()
            print("[SOCKET] Conexión cerrada.")
