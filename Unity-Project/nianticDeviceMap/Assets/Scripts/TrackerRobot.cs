using System;
using System.Collections;
using System.IO;
using Niantic.Lightship.AR.Mapping;
using Niantic.Lightship.AR.PersistentAnchors;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.ARSubsystems;

public enum RobotMode
{
    None,
    Calibration,
    Operator
}

public class TrackerRobot : MonoBehaviour
{
    [Header("Managers")]
    public ARPersistentAnchorManager persistentAnchorManager;
    public ARDeviceMappingManager deviceMappingManager;

    [Header("UI")]
    public Button startTrackingButton;

    [Header("Robot Prefabs")]
    public GameObject robotBasePrefab;
    public GameObject robotPreviewPrefab;

    [Header("Target Prefab")]
    public GameObject targetSpherePrefab;   // ← NUEVO

    private GameObject robotInstance;
    private GameObject previewInstance;
    private GameObject targetSphereInstance; // ← NUEVO

    private RobotJointController joint;
    private ARPersistentAnchor anchor;
    private ARDeviceMap deviceMap;

    [Header("Materials")]
    public Material effectorMaterial;

    [Header("IK Settings")]
    public bool autoStartIK = false;
    public float ikStepAngleDeg = 1f;
    public int ikIterationsPerFrame = 10;
    public float ikMoveDuration = 3f;

    private Coroutine ikRoutine = null;

    private Transform effectorSphereTransform; // esfera azul TCP
    public Transform targetSphereTransform => targetSphereInstance?.transform; // esfera verde (world)

    public RobotMode currentMode = RobotMode.None;
    public Action<bool> OnTrackingReady;

    public WebsocketWorker wsWorker;  // referencia al script websocketWorker

    // =======================================================================
    // START
    // =======================================================================
    private void Start()
    {
        persistentAnchorManager.DeviceMappingLocalizationEnabled = true;
        persistentAnchorManager.ContinuousLocalizationEnabled = true;
        persistentAnchorManager.TransformUpdateSmoothingEnabled = true;

        StartCoroutine(persistentAnchorManager.RestartSubsystemAsyncCoroutine());

        if (startTrackingButton)
            startTrackingButton.onClick.AddListener(StartTracking);
    }

    private void OnDestroy()
    {
        StopIKMove();
        persistentAnchorManager.arPersistentAnchorStateChanged -= OnAnchorStateChanged;

        if (startTrackingButton)
            startTrackingButton.onClick.RemoveListener(StartTracking);
    }

    // =======================================================================
    // PREVIEW SUPPORT (REQUERIDO POR AnchorPlacementController)
    // =======================================================================
    public void CreatePreviewIfNeeded(GameObject prefab)
    {
        if (!previewInstance)
            previewInstance = Instantiate(prefab);

        previewInstance.SetActive(true);
    }

    public void MovePreviewLocal(Vector3 localPos, Quaternion localRot)
    {
        if (previewInstance != null && AnchorTransform != null)
        {
            previewInstance.transform.position =
                AnchorTransform.TransformPoint(localPos);

            previewInstance.transform.rotation =
                AnchorTransform.rotation * localRot;
        }
    }

    public void HidePreview()
    {
        if (previewInstance != null)
            previewInstance.SetActive(false);
    }

    public void ApplyPreviewToRobot(Vector3 localPos, Quaternion localRot)
    {
        // Guardar la pose en disco para futuras sesiones
        AnchorPoseStorage.SaveLocalPose(localPos, localRot);

        // Ocultar preview para no dejar basura visual
        HidePreview();
    }

    // =======================================================================
    // START TRACKING
    // =======================================================================
    public void StartTracking()
    {
        currentMode = RobotMode.Operator;

        persistentAnchorManager.arPersistentAnchorStateChanged -= OnAnchorStateChanged;
        persistentAnchorManager.arPersistentAnchorStateChanged += OnAnchorStateChanged;

        LoadDeviceMapAndStartTracking();
    }

    private void LoadDeviceMapAndStartTracking()
    {
        string path = Path.Combine(Application.persistentDataPath, Mapper.MapFileName);

        if (!File.Exists(path))
        {
            Debug.LogError("❌ No se encontró el DeviceMap. Ejecuta el mapeo primero.");
            OnTrackingReady?.Invoke(false);
            return;
        }

        deviceMap = ARDeviceMap.CreateFromSerializedData(File.ReadAllBytes(path));
        Debug.Log("✔ DeviceMap cargado desde: " + path + deviceMap);
        StartCoroutine(RestartTrackingRoutine());
    }

    private IEnumerator RestartTrackingRoutine()
    {
        persistentAnchorManager.enabled = false;
        yield return null;

        persistentAnchorManager.enabled = true;
        deviceMappingManager.SetDeviceMap(deviceMap);

        bool ok = persistentAnchorManager.TryTrackAnchor(
            new ARPersistentAnchorPayload(deviceMap.GetAnchorPayload()),
            out anchor);

        if (!ok || anchor == null)
        {
            Debug.LogError("❌ No se pudo trackear el anchor.");
            OnTrackingReady?.Invoke(false);
        }
    }

    // =======================================================================
    // ANCHOR STATE
    // =======================================================================
    private void OnAnchorStateChanged(ARPersistentAnchorStateChangedEventArgs args)
    {
        if (args.arPersistentAnchor != anchor) return;
        if (anchor.trackingState != TrackingState.Tracking) return;

        Debug.Log("✔ Anchor tracking activo.");

        CreateOrPositionRobot();
        AddEffectorMarker();

        StartCoroutine(FindEffectorSphereNextFrame());

        OnTrackingReady?.Invoke(true);
    }

    // =======================================================================
    // CREATE ROBOT
    // =======================================================================
    private void CreateOrPositionRobot()
    {
        if (!robotInstance)
        {
            robotInstance = Instantiate(robotBasePrefab, anchor.transform);

            joint = robotInstance.AddComponent<RobotJointController>();
            joint.AutoDetectJoints(robotInstance.transform);
        }

        if (AnchorPoseStorage.TryLoadLocalPose(out var localPos, out var localRot))
        {
            robotInstance.transform.localPosition = localPos;
            robotInstance.transform.localRotation = localRot;
        }
        else
        {
            robotInstance.transform.localPosition = Vector3.zero;
            robotInstance.transform.localRotation = Quaternion.identity;
        }
    }

    // =======================================================================
    // EFFECTOR SPHERE (BLUE)
    // =======================================================================
    private void AddEffectorMarker()
    {
        if (joint == null || joint.endEffector == null)
            return;

        if (joint.endEffector.GetComponentInChildren<EffectorMarker>() != null)
            return;

        EffectorMarker marker = joint.endEffector.gameObject.AddComponent<EffectorMarker>();
        marker.effectorMaterial = effectorMaterial;
    }

    private IEnumerator FindEffectorSphereNextFrame()
    {
        yield return null;

        foreach (Transform child in joint.endEffector.GetComponentsInChildren<Transform>())
        {
            if (child.name.Contains("EffectorSphere"))
            {
                effectorSphereTransform = child;
                Debug.Log("✔ EffectorSphere detectada.");
                break;
            }
        }
    }

    // =======================================================================
    // TARGET SPHERE (GREEN) - WORLD POSITION
    // =======================================================================
    public void PlaceTargetAt(Vector3 worldPos)
    {
        CreateTargetSphereWorld();

        targetSphereInstance.transform.position = worldPos;

        Debug.Log($"✔ Nuevo target colocado en: {worldPos}");
        StartCoroutine(DebugAfterPlacement());

        if (autoStartIK)
            StartIKMoveToTarget();
    }

    private IEnumerator DebugAfterPlacement()
    {
        for (int i = 0; i < 10; i++)
        {
            Debug.Log($"Frame {i} target pos: {targetSphereInstance.transform.position}");
            yield return null;
        }
    }

    private void CreateTargetSphereWorld()
    {
        if (targetSphereInstance == null)
        {
            targetSphereInstance = Instantiate(targetSpherePrefab);
            targetSphereInstance.transform.localScale = Vector3.one * 0.08f;
        }
    }

    // =======================================================================
    // IK MOVE
    // =======================================================================
    public void StartIKMoveToTarget()
    {
        if (joint == null || targetSphereInstance == null)
        {
            Debug.LogWarning("No se puede pedir IK: faltan referencias");
            return;
        }

        // ---- 1) POSICIÓN EN UNITY WORLD ----
        Vector3 p_world = targetSphereInstance.transform.position;
        Quaternion r = targetSphereInstance.transform.rotation;

        // ---- 2) POSICIÓN DESDE LA BASE DEL ROBOT ----
        Transform baseTf = joint.link1.parent;
        Vector3 p_base = baseTf.InverseTransformPoint(p_world);

        // ---- 3) CONVERSIÓN UNIDAD UNITY → UR ----
        Vector3 p_ur;
        p_ur.x = p_base.z;
        p_ur.y = p_base.x;
        p_ur.z = p_base.y;

        // LOGS PARA COMPARAR EN PYTHON
        Debug.Log($"[UNITY] Target World = {p_world}");
        Debug.Log($"[UNITY] Target Local (Base) = {p_base}");
        Debug.Log($"[UNITY] Target Converted (UR frame) = {p_ur}");

        // Mensaje simple en JSON
        // ---- 4) ENVIAR A PYTHON ----
        IKRequestMsg msg = new IKRequestMsg();
        msg.type = "ik_request";
        msg.position = new float[] { p_ur.x, p_ur.y, p_ur.z };
        msg.rotation = new float[] { 0, 0, 0, 1 };
        string json = JsonUtility.ToJson(msg);

        wsWorker.SendCommand(json);

        Debug.Log($"[UNITY] JSON enviado a Python: {json}");
    }

    [System.Serializable]
    public class IKRequestMsg
    {
        public string type;
        public float[] position;
        public float[] rotation;
    }

    // Este método lo llamará tu cliente WebSocket cuando Python
    // devuelva la solución de IK:
    public void OnIkSolutionReceived(float[] q)
    {
        if (joint != null && q != null && q.Length == 6)
            joint.ApplyJointAngles(q);
    }

    [System.Serializable]
    public class BasePoseReport
    {
        public string type = "base_pose";
        public float[] position;
        public float[] rotation;
    }

    public void SendRobotBasePoseToPython()
    {
        if (joint == null)
        {
            Debug.LogWarning("[TrackerRobot] No hay joint controller.");
            return;
        }

        Transform baseTf = joint.link1.parent;

        Vector3 p_world = baseTf.position;
        Quaternion q_world = baseTf.rotation;

        BasePoseReport msg = new BasePoseReport
        {
            position = new float[] { p_world.x, p_world.y, p_world.z },
            rotation = new float[] { q_world.x, q_world.y, q_world.z, q_world.w }
        };

        string json = JsonUtility.ToJson(msg);
        wsWorker.SendCommand(json);

        Debug.Log("[UNITY] Envié base_pose → Python: " + json);
    }



    [System.Serializable]
    public class DigitalTCPReport
    {
        public string type;
        public float[] position;
        public float[] rotation;
    }

    public void SendCurrentDigitalTCPToPython()
    {
        if (effectorSphereTransform == null || joint == null)
        {
            Debug.LogWarning("[TrackerRobot] No hay effector o joint.");
            return;
        }

        // 1. World space
        Vector3 p_world = effectorSphereTransform.position;

        // 2. Local relative to base
        Transform baseTf = joint.link1.parent;
        Vector3 p_local = baseTf.InverseTransformPoint(p_world);

        // 3. Convert local → UR frame (usa tu conversión actual)
        Vector3 p_ur;
        p_ur.x = p_local.z;
        p_ur.y = -p_local.x;   // o sin el menos, según calibración
        p_ur.z = p_local.y;

        // Debug.Log($"[UNITY TCP] Digital TCP world = {p_world}");
        // Debug.Log($"[UNITY TCP] Digital TCP local = {p_local}");
        // Debug.Log($"[UNITY TCP] Digital TCP URframe = {p_ur}");

        Quaternion q_world = effectorSphereTransform.rotation;

        // convertir a marco local de la base
        Quaternion q_local = Quaternion.Inverse(baseTf.rotation) * q_world;

        // convertir a rotación UR (porque Unity usa Y-up)
        Quaternion q_ur = new Quaternion(q_local.z, -q_local.x, q_local.y, q_local.w); // misma conversión de pos

        DigitalTCPReport msg = new DigitalTCPReport {
            type = "digital_tcp",
            position = new float[]{ p_ur.x, p_ur.y, p_ur.z },
            rotation = new float[]{ q_ur.x, q_ur.y, q_ur.z, q_ur.w }
        };

        string json = JsonUtility.ToJson(msg);
        wsWorker.SendCommand(json);

        Debug.Log("[UNITY] Envié digital_tcp → Python");
    }
    private void StopIKMove()
    {
        if (ikRoutine != null)
            StopCoroutine(ikRoutine);

        ikRoutine = null;
    }

    // =======================================================================
    // UTILS
    // =======================================================================
    public bool HasAnchorTracking()
    {
        return anchor != null && anchor.trackingState == TrackingState.Tracking;
    }

    public Transform AnchorTransform =>
        anchor != null ? anchor.transform : null;

    public Transform RobotInstanceTransform =>
        robotInstance != null ? robotInstance.transform : null;
}
