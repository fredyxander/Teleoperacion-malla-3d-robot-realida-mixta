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
