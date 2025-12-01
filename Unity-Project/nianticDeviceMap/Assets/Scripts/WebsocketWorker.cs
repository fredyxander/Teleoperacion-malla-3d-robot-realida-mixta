using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using NativeWebSocket;

public class WebsocketWorker : MonoBehaviour
{
    WebSocket ws;
    private bool cancelConnection = false;   // Flag para cancelar intento

    [Header("WS URL CONFIG")]
    public string url;                         // URL WebSocket server
    public TMP_InputField urlInputField;       // Campo donde el usuario ingresa la URL

    [Header("UI Status")]
    public TextMeshProUGUI connectionStatusText;     // Texto estado de conexión del servidor
    public RobotCommands uiRobotManager;  // referencia al script RobotCommands

    // Actualiza la URL desde el InputField (SOLO INTERNO)
    private void UpdateURLFromInput()
    {
        if (urlInputField != null)
        {
            url = urlInputField.text.Trim();

            if (string.IsNullOrEmpty(url))
            {
                Debug.LogWarning("URL vacía, no se puede conectar.");
            }
            else
            {
                Debug.Log("URL WebSocket actualizada a: " + url);
            }
        }
    }

    void Start()
    {
        if (urlInputField != null)
        {
            urlInputField.text = url; // <-- sincroniza inspector → UI
        }

        UpdateConnectionStatus("Robot Desconectado", Color.gray);
    }

    //conectar websocket server
    public void ConnectToServer()
    {
        StartCoroutine(ConnectRoutine());
    }

    //Cancelar conexión websocket
    public void CancelConnectionAttempt()
    {
        Debug.Log("Cancelando intento de conexión...");
        cancelConnection = true;

        if (ws != null)
        {
            ws.Close();
            ws = null;
        }

        UpdateConnectionStatus("Cancelando Conexión...", Color.red);
        // Ejecutar timeout de 2 segundos
        StartCoroutine(CancelTimeoutRoutine());
    }

    private IEnumerator CancelTimeoutRoutine()
    {
        yield return new WaitForSeconds(2f);

        UpdateConnectionStatus("Estado: Desconectado", Color.white);
        Debug.Log("Estado restablecido después del timeout de cancelación.");
    }


    // Conexión al servidor websocket - envio - recibe
    private IEnumerator ConnectRoutine()
    {
        cancelConnection = false; // reset

        // Primero actualizamos la URL automáticamente
        UpdateURLFromInput();

        if (string.IsNullOrEmpty(url))
        {
            UpdateConnectionStatus("URL inválida", Color.red);
            yield break;
        }

        UpdateConnectionStatus("Conectando Robot...", Color.yellow);
        
        ws = new WebSocket(url);

        ws.OnOpen += () =>
        {
            Debug.Log("Conectado al servidor WebSocket");
            UpdateConnectionStatus("Robot Conectado", Color.green);
        };

        ws.OnMessage += (bytes) =>
        {
            if (cancelConnection) return;

            string msg = System.Text.Encoding.UTF8.GetString(bytes);
            Debug.Log("Mensaje recibido desde server py: " + msg);


            if (msg == "home_reached")
            {
                uiRobotManager.UpdateJointStatus("Robot in Home", Color.green);
            }

        };

        ws.OnError += (e) =>
        {
            if (cancelConnection) return;

            Debug.LogError("Error WebSocket: " + e);
            UpdateConnectionStatus("Robot: Error de conexión", Color.red);
        };

        ws.OnClose += (e) =>
        {
            if (cancelConnection) return;

            Debug.Log("Conexión cerrada");
            UpdateConnectionStatus("Robot Desconectado", Color.gray);
        };

        ws.Connect();

        // Esperar mientras intenta conectar
        float timeout = 8f;
        float timer = 0f;

        while (ws != null && ws.State == WebSocketState.Connecting)
        {
            if (cancelConnection)
                yield break;

            timer += Time.deltaTime;

            if (timer >= timeout)
            {
                Debug.LogWarning("Timeout intentando conectar");
                CancelConnectionAttempt();
                yield break;
            }

            yield return null;
        }
    }

    // Método para desconectar servidor websocket
    public async void DisconnectFromServer()
    {
        cancelConnection = true;

        if (ws != null && ws.State == WebSocketState.Open)
        {
            await ws.Close();
            Debug.Log("Desconectado manualmente.");
            UpdateConnectionStatus("Robot Desconectado", Color.gray);
        }

        ws = null;
    }

    // Actualiza el texto de conexión del robot en pantalla
    private void UpdateConnectionStatus(string message, Color color)
    {
        if (connectionStatusText != null)
        {
            connectionStatusText.text = message;
            connectionStatusText.color = color;
        }
    }

    //Metodo para enviar comando a robot
    public async void SendCommand(string cmd)
    {
        if (ws != null && ws.State == WebSocketState.Open)
        {
            await ws.SendText(cmd);
            Debug.Log($"Enviando comando: {cmd}");
        }
        else
        {
            Debug.LogWarning("No conectado al servidor WebSocket");
            UpdateConnectionStatus("Robot desconectado", Color.gray);
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