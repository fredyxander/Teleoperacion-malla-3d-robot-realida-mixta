using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;   // Necesario para TextMeshPro
using WebSocketSharp;
public class websocketWorker : MonoBehaviour
{
    public string url;
    WebSocket ws;
    public bool IsConnected => ws != null && ws.IsAlive;
    [SerializeField] private TMP_Text conectionStatus;   // Arrastra tu objeto TextMeshPro en el inspector del objeto asociado a este script "WEbSocketManager"
    [SerializeField] private GameObject displayControlPanel; //GameObject de comandos
    public robotCommands uiRobotManager;  // referencia al script websocketWorker

    // Start is called before the first frame update
    void Start()
    {
        // Estado inicial por defecto
        UpdateConnectionStatus("Esperando conexión...", Color.gray);

        // Asegurarse de que el boton arranque oculto
        if (displayControlPanel != null)
            displayControlPanel.SetActive(false);// SetActive oculta/muestra gameObject
    }


    // Método que puedes llamar desde un botón
    public void ConnectToServer()
    {
        if (ws != null && ws.IsAlive)
        {
            Debug.Log("Ya estás conectado al servidor WebSocket");
            return;
        }

        ws = new WebSocket(url);

        ws.OnOpen += (sender, e) =>
        {
            Debug.Log("Conectado al servidor WebSocket");
            UpdateConnectionStatus("Conectado", Color.green);

            if (displayControlPanel != null)
                displayControlPanel.SetActive(IsConnected);
        };

        ws.OnMessage += (sender, e) =>
        {
            Debug.Log("Mensaje recibido: " + e.Data);
            if (e.Data == "limit_reached")
            {
                uiRobotManager.UpdateJointStatus("Límite alcanzado", Color.red);
            }
        };

        ws.OnClose += (sender, e) =>
        {
            Debug.LogWarning("Conexión cerrada");
            UpdateConnectionStatus("Desconectado", Color.red);

            if (displayControlPanel != null)
                displayControlPanel.SetActive(false);
        };

        ws.OnError += (sender, e) =>
        {
            Debug.LogError("Error: " + e.Message);
            UpdateConnectionStatus("Error de conexión", Color.yellow);
        };

        ws.Connect();
    }

    //Metodo para enviar comando a robot
    public void SendCommand(string cmd)
    {
        if (ws != null && ws.IsAlive)
        {
            ws.Send(cmd);
            Debug.Log("Enviado: " + cmd);
            if(cmd != null)
            {
                uiRobotManager.UpdateJointStatus("En movimiento...", Color.green);
            }
        }
        else
        {
            Debug.LogWarning("No conectado al servidor WebSocket");
            UpdateConnectionStatus("No conectado", Color.gray);
        }
    }

    // Cerrar conexión con un botón
    public void DisconnectFromServer()
    {
        if (ws != null)
        {
            ws.Close();
            Debug.Log("Conexión cerrada manualmente");
            UpdateConnectionStatus("Desconectado", Color.red);

            if (displayControlPanel != null)
                displayControlPanel.SetActive(false);
        }
    }

    // Actualiza el texto de conexión del robot en pantalla
    private void UpdateConnectionStatus(string message, Color color)
    {
        if (conectionStatus != null)
        {
            conectionStatus.text = "Conexión Robot: " + message;
            conectionStatus.color = color;
        }
    }
}
