using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

public class AnchorPlacementController : MonoBehaviour
{
    [Header("Refs")]
    public XRRayInteractor rayInteractor;
    public TrackerRobot tracker;

    [Header("Audio")]
    public AudioClip successDing;

    [Header("Estado interno")]
    private InputDevice rightHand;
    private bool hasRightHand = false;
    private bool lastBPressed = false;

    // Offset local temporal (preview)
    private Vector3 pendingLocalPos;
    private Quaternion pendingLocalRot;
    private bool hasPendingPose = false;

    [Header("Fine Adjustment")]
    public float positionStep = 0.001f;   // 0.1 cm = 1mm
    public float rotationStep = 1f;      // 1 grado

    void Start()
    {
        GetRightHandDevice();
    }

    void GetRightHandDevice()
    {
        var devices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.RightHand, devices);

        if (devices.Count > 0)
        {
            rightHand = devices[0];
            hasRightHand = true;
        }
    }

    void Update()
    {
        if (!tracker.HasAnchorTracking())
            return;

        if (!hasRightHand || !rightHand.isValid)
            GetRightHandDevice();

        bool bPressed = false;

        // Bot?n B
        if (hasRightHand && rightHand.TryGetFeatureValue(CommonUsages.secondaryButton, out bPressed))
        {
            if (bPressed && !lastBPressed)
            {
                OnBPressed();
            }

            lastBPressed = bPressed;
        }
    }

    public void AdjustX(float direction)
    {
        if (!hasPendingPose) return;

        pendingLocalPos.x += positionStep * direction;

        tracker.MovePreviewLocal(pendingLocalPos, pendingLocalRot);
    }

    public void AdjustY(float direction)
    {
        if (!hasPendingPose) return;

        pendingLocalPos.y += positionStep * direction;

        tracker.MovePreviewLocal(pendingLocalPos, pendingLocalRot);
    }

    public void AdjustZ(float direction)
    {
        if (!hasPendingPose) return;

        pendingLocalPos.z += positionStep * direction;

        tracker.MovePreviewLocal(pendingLocalPos, pendingLocalRot);
    }

    public void RotateYaw(float direction)
    {
        if (!hasPendingPose) return;

        pendingLocalRot *= Quaternion.Euler(0, rotationStep * direction, 0);

        tracker.MovePreviewLocal(pendingLocalPos, pendingLocalRot);
    }

    public void RotatePitch(float direction)
    {
        if (!hasPendingPose) return;

        pendingLocalRot *= Quaternion.Euler(rotationStep * direction, 0, 0);

        tracker.MovePreviewLocal(pendingLocalPos, pendingLocalRot);
    }

    public void RotateRoll(float direction)
    {
        if (!hasPendingPose) return;

        pendingLocalRot *= Quaternion.Euler(0, 0, rotationStep * direction);

        tracker.MovePreviewLocal(pendingLocalPos, pendingLocalRot);
    }

    private void OnBPressed()
    {
        if (!rayInteractor.TryGetCurrent3DRaycastHit(out RaycastHit hit))
            return;

        Transform anchor = tracker.AnchorTransform;

        // 1. POSICI?N
        pendingLocalPos = anchor.InverseTransformPoint(hit.point);

        // 2. ROTACI?N ? SIEMPRE PARALELA AL PISO
        Vector3 fwd = Camera.main.transform.forward;

        // Evitar casos donde forward es casi vertical
        if (Mathf.Abs(fwd.y) > 0.95f)
            fwd = Camera.main.transform.right; // fallback seguro

        fwd.y = 0f;          // quitar inclinaci?n
        fwd.Normalize();

        Quaternion worldRot = Quaternion.LookRotation(fwd, Vector3.up);
        pendingLocalRot = Quaternion.Inverse(anchor.rotation) * worldRot;

        // 3. ACTIVAR PREVIEW
        hasPendingPose = true;

        // Crear preview si falta
        tracker.CreatePreviewIfNeeded(tracker.robotPreviewPrefab);

        // Mover preview en local space
        tracker.MovePreviewLocal(pendingLocalPos, pendingLocalRot);
        Debug.Log($"[AnchorPlacement] Preview colocado en: {hit.point}");
    }

    // Llamado por el bot?n Confirmar en UI
    public void ConfirmPlacement()
    {
        if (!hasPendingPose)
        {
            Debug.Log("[AnchorPlacement] Nada que confirmar.");
            return;
        }

        // Reproducir sonido de confirmaci?n sin AudioSource
        if (successDing != null)
        {
            AudioSource.PlayClipAtPoint(successDing, Camera.main.transform.position, 1f);
        }

        tracker.ApplyPreviewToRobot(pendingLocalPos, pendingLocalRot);

        hasPendingPose = false;

        Debug.Log("[AnchorPlacement] Posici?n del robot confirmada y guardada.");
    }
}