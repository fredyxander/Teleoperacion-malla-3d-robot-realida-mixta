using Niantic.Lightship.AR.Mapping;
using Niantic.Lightship.AR.MapStorageAccess;
using System;
using System.Collections;
using System.IO;
using UnityEngine;
using UnityEngine.UI;

public class Mapper : MonoBehaviour
{
    public const string MapFileName = "RobotDeviceMap.bin";

    [SerializeField] private AudioSource mappingAudio;
    [SerializeField] private ARDeviceMappingManager deviceMappingManager;
    [SerializeField] private Button startMappingButton;
    [SerializeField] private float mappingSeconds = 10f;

    private void Start()
    {
        // Suscribirse al evento para saber cuándo el mapa está listo
        deviceMappingManager.DeviceMapFinalized += OnDeviceMapFinalized;

        // Click del botón
        startMappingButton.onClick.AddListener(OnStartMappingClicked);
    }

    private void OnDestroy()
    {
        deviceMappingManager.DeviceMapFinalized -= OnDeviceMapFinalized;
    }

    public void OnStartMappingClicked()
    {
        StartCoroutine(RunMapping());
    }

    private IEnumerator RunMapping()
    {
        startMappingButton.interactable = false;

        // Activar sonido de escaneo
        if (mappingAudio != null)
            mappingAudio.Play();

        // Crear mapa nuevo
        deviceMappingManager.SetDeviceMap(new ARDeviceMap());
        deviceMappingManager.StartMapping();

        // Escanear por el tiempo definido
        yield return new WaitForSeconds(mappingSeconds);

        // Parar mapeo
        deviceMappingManager.StopMapping();

        // Detener sonido
        if (mappingAudio != null)
            mappingAudio.Stop();

        startMappingButton.interactable = true;
    }

    private void OnDeviceMapFinalized(ARDeviceMap map)
    {
        if (!map.HasValidMap())
        {
            Debug.LogError("DeviceMap vacío o inválido");
            return;
        }

        var serialized = map.Serialize();
        var path = Path.Combine(Application.persistentDataPath, MapFileName);
        File.WriteAllBytes(path, serialized);
        Debug.Log($"DeviceMap guardado en: {path}");
    }
}

