using System.IO;
using UnityEngine;

[System.Serializable]
public class AnchorPoseData
{
    public float px, py, pz;
    public float rx, ry, rz, rw;

    public AnchorPoseData() { }

    public AnchorPoseData(Vector3 pos, Quaternion rot)
    {
        px = pos.x; py = pos.y; pz = pos.z;
        rx = rot.x; ry = rot.y; rz = rot.z; rw = rot.w;
    }

    public Vector3 ToPosition() => new(px, py, pz);
    public Quaternion ToRotation() => new(rx, ry, rz, rw);
}

public static class AnchorPoseStorage
{
    private static readonly string FilePath =
        Path.Combine(Application.persistentDataPath, "robotAnchorLocalPose.json");

    public static void SaveLocalPose(Vector3 pos, Quaternion rot)
    {
        var data = new AnchorPoseData(pos, rot);
        File.WriteAllText(FilePath, JsonUtility.ToJson(data));
        Debug.Log($"[AnchorPoseStorage] Guardado offset local en {FilePath}");
    }

    public static bool TryLoadLocalPose(out Vector3 pos, out Quaternion rot)
    {
        if (!File.Exists(FilePath))
        {
            pos = Vector3.zero;
            rot = Quaternion.identity;
            return false;
        }

        var json = File.ReadAllText(FilePath);
        var data = JsonUtility.FromJson<AnchorPoseData>(json);

        pos = data.ToPosition();
        rot = data.ToRotation();
        return true;
    }
}
