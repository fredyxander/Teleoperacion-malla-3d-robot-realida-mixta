using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class MappingPanel : MonoBehaviour
{
    [Header("Panel que se mostrará/ocultará")]
    public GameObject panel;
    
    [Header("Posición relativa (modo tablet)")]
    public float distance = 0.40f;       // qué tan lejos al frente
    public float horizontalOffset = -0.25f; // < 0 = a la IZQUIERDA, > 0 = derecha
    public float verticalOffset = -0.5f;   // < 0 = más ABAJO, > 0 = arriba

    [Header("Inclinación tipo tablet (grados)")]
    public float tiltXDeg = 25f;  // inclinación hacia abajo (alrededor del eje X de la cámara)
    public float tiltYDeg = -15f; // leve giro a la izquierda/derecha (alrededor del eje Y mundial)

    [Header("Suavizado de seguimiento")]
    public float followSmoothness = 10f;

    [Header("Cámara XR del usuario")]
    public Transform xrCamera;

    private InputDevice leftHand;
    private bool lastXPressed = false;
    private bool panelIsVisible = false;

    public static bool PanelIsOpen = false;

    void Start()
    {
        if (xrCamera == null && Camera.main != null)
            xrCamera = Camera.main.transform;

        GetLeftHandDevice();

        if (panel != null)
            panel.SetActive(false);
    }

    void GetLeftHandDevice()
    {
        var devices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.LeftHand, devices);

        if (devices.Count > 0)
            leftHand = devices[0];
    }

    void Update()
    {
        if (!leftHand.isValid)
            GetLeftHandDevice();

        bool xPressed = false;

        if (leftHand.TryGetFeatureValue(CommonUsages.primaryButton, out xPressed))
        {
            if (xPressed && !lastXPressed)
                TogglePanel();

            lastXPressed = xPressed;
        }
    }

    void TogglePanel()
    {
        if (panel == null || xrCamera == null)
            return;

        PanelIsOpen = !PanelIsOpen;
        panel.SetActive(PanelIsOpen);

        if (PanelIsOpen)
        {
            SnapInFrontOfUser();
        }
        else
        {
            StartCoroutine(ResetRayInteractorProperly());
        }
    }

    public void ClosePanel()
    {
        Debug.Log("click en boton");
        PanelIsOpen = false;
        panel.SetActive(false);
        StartCoroutine(ResetRayInteractorProperly());
    }

    void LateUpdate()
    {
        if (PanelIsOpen)
            FollowUserView();
    }

    void SnapInFrontOfUser()
    {
        if (panel == null) return;

        GetTargetPosAndRot(out Vector3 targetPos, out Quaternion targetRot);

        panel.transform.position = targetPos;
        panel.transform.rotation = targetRot;
    }
    void FollowUserView()
    {
        if (xrCamera == null || panel == null) return;

        GetTargetPosAndRot(out Vector3 targetPos, out Quaternion targetRot);

        // Suavizado de movimiento
        panel.transform.position = Vector3.Lerp(
            panel.transform.position,
            targetPos,
            Time.deltaTime * followSmoothness
        );

        // Suavizado de rotación
        panel.transform.rotation = Quaternion.Slerp(
            panel.transform.rotation,
            targetRot,
            Time.deltaTime * followSmoothness
        );
    }

    IEnumerator ResetRayInteractorProperly()
    {
        // 1. Buscar el ray interactor (nuevo API Unity 6)
        var ray = UnityEngine.Object.FindFirstObjectByType<
            UnityEngine.XR.Interaction.Toolkit.Interactors.XRRayInteractor>();

        if (ray != null)
        {
            // 2. Deshabilitarlo un frame
            ray.enabled = false;
        }

        // 3. Esperar 1 frame
        yield return null;

        // 4. Rehabilitarlo
        if (ray != null)
        {
            ray.enabled = true;
        }
    }

    void GetTargetPosAndRot(out Vector3 targetPos, out Quaternion targetRot)
    {
        if (xrCamera == null)
        {
            targetPos = panel.transform.position;
            targetRot = panel.transform.rotation;
            return;
        }

        // Direcciones de la cámara
        Vector3 camForward = xrCamera.forward;
        Vector3 camRight = xrCamera.right;

        // POSICIÓN:
        //   - distance al frente
        //   - offset horizontal hacia la izquierda (negativo) o derecha (positivo)
        //   - offset vertical hacia abajo (negativo) o arriba (positivo)
        targetPos =
            xrCamera.position +
            camForward * distance +
            camRight * horizontalOffset +
            Vector3.up * verticalOffset;

        // ROTACIÓN base: mirando hacia donde mira la cámara
        targetRot = Quaternion.LookRotation(camForward, Vector3.up);

        // Aplicar inclinación tipo tablet:
        //   tiltXDeg: gira alrededor del eje "right" de la cámara → como bajar la punta de la tablet
        //   tiltYDeg: gira alrededor del eje vertical mundial → leve giro hacia un lado
        targetRot = Quaternion.AngleAxis(tiltXDeg, xrCamera.right) * targetRot;
        targetRot = Quaternion.AngleAxis(tiltYDeg, Vector3.up) * targetRot;
    }
}
