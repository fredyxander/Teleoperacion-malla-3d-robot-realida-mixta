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

    // Caché del último punto estable donde golpeó el ray
    private Vector3 lastValidHitPoint;
    private bool hasLastHit = false;

    // Nuevo: detectar SOLO el primer frame del botón A
    private bool lastAPressed = false;

    void Start()
    {
        // Crear el marcador visual del raycast
        markerInstance = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        markerInstance.transform.localScale = Vector3.one * markerSize;

        var renderer = markerInstance.GetComponent<Renderer>();
        if (renderer) renderer.material.color = markerColor;

        // Quitar collider (no colisiones)
        Destroy(markerInstance.GetComponent<Collider>());

        markerInstance.SetActive(false);
    }

    void Update()
    {
        if (rayInteractor == null)
            return;

        // Raycast del XRRayInteractor
        if (rayInteractor.TryGetCurrent3DRaycastHit(out RaycastHit hit))
        {
            markerInstance.SetActive(true);
            markerInstance.transform.position = hit.point;

            // Mostrar distancia
            float dist = Vector3.Distance(rayInteractor.transform.position, hit.point);
            if (distanceText)
                distanceText.text = $"{dist:0.00} m";

            // Mostrar coordenadas
            if (coordinateText)
                coordinateText.text = $"({hit.point.x:0.00}, {hit.point.y:0.00}, {hit.point.z:0.00})";

            // Guardar el último punto estable del ray
            lastValidHitPoint = hit.point;
            hasLastHit = true;

            // ==============================================================
            // Detección del botón A SOLO en el primer frame (rising edge)
            // ==============================================================

            List<InputDevice> rightHandDevices = new List<InputDevice>();
            InputDevices.GetDevicesWithCharacteristics(
                InputDeviceCharacteristics.Right | InputDeviceCharacteristics.Controller,
                rightHandDevices
            );

            foreach (var device in rightHandDevices)
            {
                bool aPressed;

                // Detectar botón A (primaryButton)
                if (device.TryGetFeatureValue(CommonUsages.primaryButton, out aPressed))
                {
                    // Solo ejecutar cuando:
                    // A pasa de "no presionado" → "presionado"
                    if (aPressed && !lastAPressed)
                    {
                        if (trackerRobot != null && hasLastHit)
                        {
                            // Aquí usamos el hit estable ANTES del cambio de estado del XR interactor
                            trackerRobot.PlaceTargetAt(lastValidHitPoint);
                            trackerRobot.StartIKMoveToTarget();

                            Debug.Log("📌 TARGET colocado con botón A (primer frame).");
                        }
                        else if (trackerRobot == null)
                        {
                            Debug.LogWarning("RayDistanceMeter: TrackerRobot reference is missing.");
                        }
                    }

                    // Guardar estado del frame actual
                    lastAPressed = aPressed;
                }
            }
        }
        else
        {
            markerInstance.SetActive(false);
        }
    }
}
