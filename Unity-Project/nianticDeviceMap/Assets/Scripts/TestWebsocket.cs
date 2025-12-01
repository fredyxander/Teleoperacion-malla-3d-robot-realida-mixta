using NativeWebSocket;
using System;
using TMPro;
using UnityEngine;

public class TestWebsocket : MonoBehaviour
{
    WebSocket ws;
    public string url = "ws://localhost:8080";

    // 🔹 Texto en pantalla con el estado
    public TextMeshProUGUI connectionStatusText;

    void Start()
    {
        if (connectionStatusText != null)
            connectionStatusText.text = "Robot Desconectado";
    }

    public async void ConnectToServer()
    {
        if (connectionStatusText != null)
            connectionStatusText.text = "Conectando...";

        ws = new WebSocket(url);

        ws.OnOpen += () =>
        {
            Debug.Log("Conectado al servidor WebSocket");

            if (connectionStatusText != null)
                connectionStatusText.text = "Conectado";
        };

        ws.OnMessage += (bytes) =>
        {
            string msg = System.Text.Encoding.UTF8.GetString(bytes);
            Debug.Log("Mensaje recibido desde server py: " + msg);
        };

        ws.OnError += (e) =>
        {
            Debug.LogError("Error WebSocket: " + e);

            if (connectionStatusText != null)
                connectionStatusText.text = "Error de conexión";
        };

        ws.OnClose += (e) =>
        {
            Debug.Log("Conexión cerrada");

            if (connectionStatusText != null)
                connectionStatusText.text = "Robot Desconectado";
        };

        await ws.Connect();
    }

    // 🔹 Método para desconectar servidor websocket
    public async void DisconnectFromServer()
    {
        if (ws != null && ws.State == WebSocketState.Open)
        {
            await ws.Close();
            Debug.Log("Desconectado manualmente.");

            if (connectionStatusText != null)
                connectionStatusText.text = "Robot Desconectado";
        }
    }

    public async void SendWSMessage(string message)
    {
        if (ws != null && ws.State == WebSocketState.Open)
        {
            await ws.SendText(message);
        }
    }

    private void Update()
    {
#if !UNITY_WEBGL || UNITY_EDITOR
        if (ws != null)
        {
            ws.DispatchMessageQueue();
        }
#endif
    }

    private async void OnApplicationQuit()
    {
        if (ws != null)
        {
            await ws.Close();
        }
    }
}
