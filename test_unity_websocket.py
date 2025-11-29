import asyncio
import websockets

connected_clients = set()

async def handle_client(websocket, path=None):
    client = websocket.remote_address
    print(f"[WS] Cliente conectado: {client}")

    connected_clients.add(websocket)

    # 🔹 ENVIAR MENSAJE AUTOMÁTICO A UNITY
    await websocket.send("Hola desde Python: conexión establecida")

    try:
        async for message in websocket:
            print(f"[WS] Mensaje recibido desde Unity: {message}")

    except websockets.exceptions.ConnectionClosedOK:
        print(f"[WS] Cliente desconectado limpiamente: {client}")

    except Exception as e:
        print(f"[WS] Error en comunicación: {e}")

    finally:
        if websocket in connected_clients:
            connected_clients.remove(websocket)
        print(f"[WS] Conexión finalizada: {client}")

async def main():
    host = "0.0.0.0"
    port = 8080

    print("=====================================")
    print("🟢 Servidor WebSocket (modo mínimo)")
    print(f"   Dirección: ws://localhost:{port}")
    print("=====================================")

    async with websockets.serve(handle_client, host, port):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())
