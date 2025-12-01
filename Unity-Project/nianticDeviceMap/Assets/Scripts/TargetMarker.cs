using UnityEngine;

public class TargetMarker : MonoBehaviour
{
    public Material targetMaterial;
    public Vector3 localOffset = new Vector3(0.25f, -0.23f, 0f); // derecha, abajo
    public float sphereSize = 0.04f; // 4 cm

    void Start()
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.name = "EffectorTargetSphere";

        sphere.transform.SetParent(transform, false);
        sphere.transform.localPosition = localOffset;
        sphere.transform.localScale = Vector3.one * sphereSize;

        var renderer = sphere.GetComponent<Renderer>();
        if (targetMaterial != null)
            renderer.material = targetMaterial;
        else
            renderer.material.color = Color.green; // fallback
        renderer.material = targetMaterial;

        // Mantener collider para raycast
        SphereCollider col = sphere.GetComponent<SphereCollider>();
        col.radius = 0.5f;
    }
}
