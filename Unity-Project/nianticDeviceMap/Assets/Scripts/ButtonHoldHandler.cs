using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;

public class ButtonHoldHandler : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    [Header("Evento continuo (con parámetro)")]
    public UnityEvent<float> onHold;   // recibe +1 o -1

    [Header("Configuración")]
    public float repeatRate = 20f;   // repeticiones por segundo

    public float holdValue = 1f;     // parámetro a enviar (por ejemplo +1 o -1)

    private bool isHeld = false;
    private float timer = 0f;

    public void OnPointerDown(PointerEventData eventData)
    {
        isHeld = true;
        timer = 0f;
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        isHeld = false;
    }

    void Update()
    {
        if (isHeld)
        {
            timer += Time.deltaTime;

            float interval = 1f / repeatRate;

            while (timer >= interval)
            {
                timer -= interval;
                onHold?.Invoke(holdValue); // ← ENVÍA EL PARÁMETRO
            }
        }
    }
}
