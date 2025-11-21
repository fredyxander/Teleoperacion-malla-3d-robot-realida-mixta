using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;

public class robotCommands : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    [SerializeField] private TMP_Text robotJoinStatusText;

    public websocketWorker wsWorker;  // referencia al script websocketWorker
    public string commandOnPress;     // comando a enviar al presionar (ej: "girar_izquierda")

    public Renderer objectBaseJoin;
    public Renderer objectShoulderJoin;
    public Renderer objectElbowJoin;
    public Renderer objectWrist1;
    public Renderer objectWrist2;
    public Renderer objectWrist3;

    public void UpdateJointStatus(string message, Color color)
    {
        if (robotJoinStatusText != null)
        {
            robotJoinStatusText.text = message;
            robotJoinStatusText.color = color;
        }
    }
    public void OnPointerDown(PointerEventData eventData)
    {
        if (wsWorker != null)
        {
            wsWorker.SendCommand(commandOnPress);
            Debug.Log("Enviado (DOWN): " + commandOnPress);
        }
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        if (wsWorker != null)
        {
            wsWorker.SendCommand("stop");
            Debug.Log("Enviado (UP): stop");
        }
    }

    public void girarBaseHover()
    {
        UpdateJointStatus("base join", Color.white);
    }

    public void girarHombroHover()
    {
        UpdateJointStatus("hombro join", Color.white);
    }
    public void girarCodoHover()
    {
        UpdateJointStatus("codo join", Color.white);
    }

    public void girarMuneca1Hover()
    {
        UpdateJointStatus("muneca1 join", Color.white);
    }

    public void girarMuneca2Hover()
    {
        UpdateJointStatus("muneca2 join", Color.white);
    }

    public void girarMuneca3Hover()
    {
        UpdateJointStatus("muneca3 join", Color.white);
    }

    public void homeHover()
    {
        UpdateJointStatus("home", Color.white);
    }

    public void pointerOut()
    {
        UpdateJointStatus("", Color.white);
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (objectBaseJoin) { objectBaseJoin.material.color = Color.gray; }
        if (objectShoulderJoin) { objectShoulderJoin.material.color = Color.gray; }
        if (objectElbowJoin) { objectElbowJoin.material.color = Color.gray; }
        if (objectWrist1) { objectWrist1.material.color = Color.gray; }
        if (objectWrist2) { objectWrist2.material.color = Color.gray; }
        if (objectWrist3) { objectWrist3.material.color = Color.gray; }

        switch (robotJoinStatusText.text)
        {
            case "base join":
                if(objectBaseJoin) objectBaseJoin.material.color= Color.green;
                break;

            case "hombro join":
                if (objectShoulderJoin) objectShoulderJoin.material.color = Color.green;
                break;

            case "codo join":
                if (objectElbowJoin) objectElbowJoin.material.color = Color.green;
                break;

            case "muneca1 join":
                if (objectWrist1) objectWrist1.material.color = Color.green;
                break;

            case "muneca2 join":
                if (objectWrist2) objectWrist2.material.color = Color.green;
                break;

            case "muneca3 join":
                if (objectWrist3) objectWrist3.material.color = Color.green;
                break;

            default:
                Debug.LogWarning("Item no reconocido:" +  robotJoinStatusText.text);
                break;
        }
    }
}
