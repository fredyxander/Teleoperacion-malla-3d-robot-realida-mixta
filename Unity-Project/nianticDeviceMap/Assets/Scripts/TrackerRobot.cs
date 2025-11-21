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

        if (autoStartIK)
            StartIKMoveToTarget();
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
        if (joint == null ||
            effectorSphereTransform == null ||
            targetSphereTransform == null)
        {
            Debug.LogWarning("No se puede iniciar IK: faltan referencias.");
            return;
        }

        if (ikRoutine != null)
            StopCoroutine(ikRoutine);

        ikRoutine = StartCoroutine(IKMoveEffectorToTarget());
    }

    private IEnumerator IKMoveEffectorToTarget()
    {
        Vector3 startPos = effectorSphereTransform.position;
        Vector3 goalPos = targetSphereTransform.position;

        float elapsed = 0f;

        while (elapsed < ikMoveDuration)
        {
            float t = Mathf.Clamp01(elapsed / ikMoveDuration);
            Vector3 intermediate = Vector3.Lerp(startPos, goalPos, t);

            joint.SolveIKToTarget(intermediate, ikIterationsPerFrame, ikStepAngleDeg);

            elapsed += Time.deltaTime;
            yield return null;
        }

        joint.SolveIKToTarget(goalPos, ikIterationsPerFrame * 3, ikStepAngleDeg * 0.5f);
        ikRoutine = null;
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
