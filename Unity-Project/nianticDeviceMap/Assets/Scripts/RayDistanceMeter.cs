//using UnityEngine;
//using UnityEngine.XR.Interaction.Toolkit;
//using TMPro;
//using UnityEngine.XR.Interaction.Toolkit.Interactors;

//public class RayDistanceMeter : MonoBehaviour
//{
//    [Header("XR")]
//    public UnityEngine.XR.Interaction.Toolkit.Interactors.XRRayInteractor rayInteractor;

//    [Header("Visual")]
//    public float markerSize = 0.04f;
//    public Color markerColor = Color.red;
//    public Color textColor = Color.white;
//    public float textFontSize = 0.5f;
//    private const float NormalOffset = 0.01f; // evita z-fighting sin atraer nada

//    private GameObject markerInstance;
//    private TextMeshPro textInstance;
//    private Transform cameraTransform;

//    // Start is called once before the first execution of Update after the MonoBehaviour is created
//    void Start()
//    {
//        if(rayInteractor == null)
//        {
//            rayInteractor = GetComponent<XRRayInteractor>();
//            if(rayInteractor == null)
//            {
//                Debug.LogError("[RayDistanceMarker] No se encontró XRRayInteractor.");
//                enabled = false;
//                return;
//            }
//        }

//        // Buscar cámara (la del XR Origin normalmente tiene el tag MainCamera)
//        if (Camera.main != null)
//        {
//            cameraTransform = Camera.main.transform;
//        }

//        // Marcador visual fijo (sin Rigidbody ni Animator)
//        markerInstance = GameObject.CreatePrimitive(PrimitiveType.Sphere);
//        markerInstance.transform.localScale = Vector3.one * markerSize;
//        var r = markerInstance.GetComponent<Renderer>();
//        if (r) r.material.color = Color.red;
//        var col = markerInstance.GetComponent<Collider>();
//        if (col) Destroy(col); // no necesitamos colision
//        markerInstance.SetActive(false);

//        // Texto en world space, solo rota (no se desplaza hacia la camara)
//        GameObject textObj = new GameObject("RayHitLabel");
//        textInstance = textObj.AddComponent<TextMeshPro>();
//        textInstance.fontSize = textFontSize;
//        textInstance.color = Color.white;
//        textInstance.alignment = TextAlignmentOptions.Center;
//        textInstance.enableAutoSizing = false;
//        textInstance.text = "";
//        textInstance.gameObject.SetActive(false);

//        Debug.Log("[RayDistanceMarker] Inicializado correctamente.");
//    }

//    void Update()
//    {
//        if (rayInteractor == null)
//            return;

//        //// 1) Si el panel está abierto Y el ray está tocando UI → NO medir
//        if (MappingPanel.PanelIsOpen &&
//            rayInteractor.TryGetCurrentUIRaycastResult(out var uiHit))
//        {
//            if (markerInstance != null) markerInstance.SetActive(false);
//            if (textInstance != null) textInstance.gameObject.SetActive(false);
//            return;
//        }

//        // 2) Medición normal sobre la malla / objetos 3D
//        if (rayInteractor.TryGetCurrent3DRaycastHit(out RaycastHit hit))
//        {
//            float distance = hit.distance;

//            Vector3 hitPos = hit.point + hit.normal * NormalOffset;

//            // Marcador
//            markerInstance.transform.position = hitPos;
//            markerInstance.transform.rotation = Quaternion.identity;
//            markerInstance.SetActive(true);

//            // Texto
//            Vector3 textPos = hitPos + Vector3.up * 0.05f;
//            textInstance.transform.position = textPos;

//            textInstance.text =
//                $"{distance:F2} m\n({hitPos.x:F2}, {hitPos.y:F2}, {hitPos.z:F2})";

//            if (cameraTransform != null)
//            {
//                textInstance.transform.LookAt(cameraTransform);
//                textInstance.transform.Rotate(0, 180f, 0);
//            }

//            if (!textInstance.gameObject.activeSelf)
//                textInstance.gameObject.SetActive(true);
//        }
//        else
//        {
//            if (markerInstance != null)
//                markerInstance.SetActive(false);

//            if (textInstance != null)
//                textInstance.gameObject.SetActive(false);
//        }
//    }
//}

using UnityEngine;
using TMPro;
using System.Collections.Generic;
using UnityEngine.XR;

public class RayDistanceMeter : MonoBehaviour
{
    [Header("XR Ray Interactor")]
    public UnityEngine.XR.Interaction.Toolkit.Interactors.XRRayInteractor rayInteractor;

    [Header("Visual")]
    public float markerSize = 0.04f;
    public Color markerColor = Color.magenta;

    [Header("Tracker Robot Reference")]
    public TrackerRobot trackerRobot;

    [Header("UI Output")]
    public TextMeshPro distanceText;
    public TextMeshPro coordinateText;

    private GameObject markerInstance;

    void Start()
    {
        // Crear la esfera que marca donde golpea el raycast
        markerInstance = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        markerInstance.transform.localScale = Vector3.one * markerSize;

        var renderer = markerInstance.GetComponent<Renderer>();
        if (renderer) renderer.material.color = markerColor;

        // Eliminar collider porque no queremos colisiones
        Destroy(markerInstance.GetComponent<Collider>());

        markerInstance.SetActive(false);
    }

    void Update()
    {
        if (rayInteractor == null)
            return;

        // Hacer raycast desde XRRayInteractor
        // 2) Medición normal sobre la malla / objetos 3D
        if (rayInteractor.TryGetCurrent3DRaycastHit(out RaycastHit hit))
        {
            markerInstance.SetActive(true);
            markerInstance.transform.position = hit.point;

            float dist = Vector3.Distance(rayInteractor.transform.position, hit.point);

            if (distanceText)
                distanceText.text = $"{dist:0.00} m";

            if (coordinateText)
                coordinateText.text = $"({hit.point.x:0.00}, {hit.point.y:0.00}, {hit.point.z:0.00})";

            // ===========================================================
            // NUEVO — Detectar botón A del control derecho (primaryButton)
            // ===========================================================
            List<InputDevice> rightHandDevices = new List<InputDevice>();
            InputDevices.GetDevicesWithCharacteristics(
                InputDeviceCharacteristics.Right | InputDeviceCharacteristics.Controller,
                rightHandDevices
            );

            foreach (var device in rightHandDevices)
            {
                // Detectar botón A (primaryButton)
                if (device.TryGetFeatureValue(CommonUsages.primaryButton, out bool aPressed) && aPressed)
                {
                    if (trackerRobot != null)
                    {
                        trackerRobot.PlaceTargetAt(hit.point);
                        trackerRobot.StartIKMoveToTarget();   // ← Ejecutar IK inmediatamente
                    }
                    else
                    {
                        Debug.LogWarning("RayDistanceMeter: TrackerRobot reference missing.");
                    }

                    break;
                }
            }
        }
        else
        {
            markerInstance.SetActive(false);
        }
    }
}

