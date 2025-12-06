using NativeWebSocket;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using UnityEngine;

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
    public RobotJointController robotJointController; //referencia al script RobotJoinController

    // Referencia opcional (si quieres arrastrarlo desde Unity)
    public TrackerRobot tracker;

    [System.Serializable]
    public class RobotStateMessage
    {
        public string type;
        public float[] tcp_pos;
        public float[] tcp_rotvec;
        public float[] joints;
    }


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

            var tracker = UnityEngine.Object.FindFirstObjectByType<TrackerRobot>();
            if (tracker != null)
            {
                tracker.SendRobotBasePoseToPython();      // 1) base pose
                tracker.SendCurrentDigitalTCPToPython();  // 2) digital tcp
            }
            else
            {
                Debug.LogWarning("[WS] No encontré TrackerRobot para enviar TCP/base pose.");
            }
        };

        ws.OnMessage += (bytes) =>
        {
            if (cancelConnection) return;

            string msg = System.Text.Encoding.UTF8.GetString(bytes);
            Debug.Log("Mensaje recibido desde server py: " + msg);

            HandleIncomingMessage(msg);
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

        // Este Update() especial SÍ es necesario para Quest
        InvokeRepeating(nameof(DispatchMessages), 0f, 0.01f);

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

    private void DispatchMessages()
    {
        ws?.DispatchMessageQueue();
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
        Debug.Log($"sending {ws}");
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

    // =====================================================
    //           PARSE DE MENSAJES DESDE PYTHON
    // =====================================================
    private void HandleIncomingMessage(string msg)
    {
        if (msg == "home_reached")
        {
            uiRobotManager.UpdateJointStatus("Robot in Home", Color.green);
            return;
        }

        try
        {
            // Intentar parsear IK solution
            RobotStateMessage data = JsonUtility.FromJson<RobotStateMessage>(msg);
            if (data != null)
            {
                if (tracker == null)
                    tracker = UnityEngine.Object.FindFirstObjectByType<TrackerRobot>();

                // Aplicar Joints
                if (robotJointController == null)
                    robotJointController = UnityEngine.Object.FindFirstObjectByType<RobotJointController>();

                if (data.type == "joint_fixed_pose")
                {
                    float[] q = data.joints;
                    Debug.Log($"[UNITY] Joint state recibido desde Python {q}");

                    robotJointController.ApplyJointAngles(q);
                    return;
                }

                Debug.Log($"Solucion IK recibida {data}");
                if (data.type == "robot_state")
                {
                    Debug.Log($"[UNITY] TCP real: ({data.tcp_pos[0]}, {data.tcp_pos[1]}, {data.tcp_pos[2]})");
                    Debug.Log($"[UNITY] data.joints: ({data.joints[0]}, {data.joints[1]}, {data.joints[2]}, {data.joints[3]}, {data.joints[4]}, {data.joints[5]})");
                }

                //Mover juntas del robot (gemelo digital)
                Debug.Log($"robotJointController: {robotJointController}");
                robotJointController.ApplyJointAngles(data.joints);

                // Actualizar esfera del TCP
                tracker.UpdateEffectorSphereFromUR(data.tcp_pos);

                Debug.Log("[UNITY] Robot actualizado desde Python.");

                return;
            }

            //// Intentar parsear IK solution
            //IKSolution data = JsonUtility.FromJson<IKSolution>(msg);

            //if (data != null && data.type == "ik_solution")
            //{
            //    Debug.Log("$[WS] Solución IK recibida {data}");

            //    if (data.type == "robot_tcp")
            //    {
            //        Debug.Log($"[UNITY] TCP real: ({data.tcp_pos[0]}, {data.tcp_pos[1]}, {data.tcp_pos[2]})");
            //    }

            //    if (tracker == null)
            //        tracker = UnityEngine.Object.FindFirstObjectByType<TrackerRobot>();

            //    if (tracker != null)
            //        tracker.OnIkSolutionReceived(data.q);

            //    return;
            //}
        }
        catch (Exception ex)
        {
            Debug.LogWarning("[WS] No se pudo parsear JSON: " + ex);
        }
    }

    // =====================================================
    //                   CLASES JSON
    // =====================================================
    [Serializable]
    public class IKSolution
    {
        public string type;
        public float[] q;
        public float[] tcp_pos;
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