using UnityEngine;

public class EffectorMarker : MonoBehaviour
{
    public Material effectorMaterial;   // ← MATERIAL COMO TARGET

    void Start()
    {
        // Crear esfera
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.name = "EffectorSphere";

        // Hacerla hija del efector (Link6)
        sphere.transform.SetParent(transform, false);

        // Posición y tamaño
        sphere.transform.localPosition = new Vector3(0.020f, 0.025f, 0f);
        sphere.transform.localScale = Vector3.one * 0.05f;

        // Asignar material EXACTO como lo hicimos con TargetSphere
        var renderer = sphere.GetComponent<Renderer>();
        if (effectorMaterial != null)
            renderer.material = effectorMaterial;
        else
            renderer.material.color = Color.blue; // fallback

        // Collider
        SphereCollider col = sphere.GetComponent<SphereCollider>();
        col.radius = 0.5f;
    }
}
