// test cinematica inversa
using UnityEngine;
using System.Collections;

public class RobotJointController : MonoBehaviour
{
    [Header("UR3 Links (autodetect)")]
    public Transform link1;
    public Transform link2;
    public Transform link3;
    public Transform link4;
    public Transform link5;
    public Transform link6;

    [Header("End Effector (opcional, por defecto Link6)")]
    public Transform endEffector;

    private float[] current = new float[6];
    private Coroutine moveRoutine = null;

    // ============================================================
    // AUTO-DETECCIÓN DE JERARQUÍA DEL UR3
    // ============================================================
    public void AutoDetectJoints(Transform robotRoot)
    {
        Transform baseNode = robotRoot.Find("UR3_Robot_model/Base");

        link1 = baseNode.Find("Link1");
        link2 = link1.Find("Link2");
        link3 = link2.Find("Link3");
        link4 = link3.Find("Link4");
        link5 = link4.Find("Link5");
        link6 = link5.Find("Link6");

        if (endEffector == null)
            endEffector = link6;

        Debug.Log("UR3 joints autodetect OK.");
        UpdateCurrentFromTransforms();
    }

    // ============================================================
    // APLICAR ÁNGULOS A LOS JOINTS (MODO JOINT SPACE)
    // ============================================================
    public void ApplyJointAngles(float[] q)
    {
        if (q == null || q.Length < 6) return;

        // Guardar estado actual
        for (int i = 0; i < 6; i++)
            current[i] = q[i];

        if (link1) link1.localRotation = Quaternion.Euler(0, q[0], 0); // Joint1 → eje Y
        if (link2) link2.localRotation = Quaternion.Euler(0, 0, q[1]); // Joint2 → eje Z
        if (link3) link3.localRotation = Quaternion.Euler(0, 0, q[2]); // Joint3 → eje Z
        if (link4) link4.localRotation = Quaternion.Euler(0, 0, q[3]); // Joint4 → eje Z
        if (link5) link5.localRotation = Quaternion.Euler(0, q[4], 0); // Joint5 → eje Y
        if (link6) link6.localRotation = Quaternion.Euler(0, 0, q[5]); // Joint6 → eje Z
    }

    public float[] GetCurrentAngles() => current;

    private void UpdateCurrentFromTransforms()
    {
        if (link1) current[0] = NormalizeAngle(link1.localEulerAngles.y);
        if (link2) current[1] = NormalizeAngle(link2.localEulerAngles.z);
        if (link3) current[2] = NormalizeAngle(link3.localEulerAngles.z);
        if (link4) current[3] = NormalizeAngle(link4.localEulerAngles.z);
        if (link5) current[4] = NormalizeAngle(link5.localEulerAngles.y);
        if (link6) current[5] = NormalizeAngle(link6.localEulerAngles.z);
    }

    private float NormalizeAngle(float angle)
    {
        angle %= 360f;
        if (angle > 180f) angle -= 360f;
        return angle;
    }

    // ============================================================
    // MOVER HACIA UNA POSE EN UN TIEMPO (INTERPOLACIÓN SUAVE)
    // ============================================================
    public void MoveToPose(float[] target, float duration)
    {
        if (moveRoutine != null)
            StopCoroutine(moveRoutine);

        moveRoutine = StartCoroutine(MoveRoutine(target, duration));
    }

    private IEnumerator MoveRoutine(float[] target, float duration)
    {
        float[] start = GetCurrentAngles();
        float t = 0f;

        while (t < duration)
        {
            t += Time.deltaTime;
            float k = Mathf.Clamp01(t / duration);

            float[] pose = new float[6];
            for (int i = 0; i < 6; i++)
                pose[i] = Mathf.Lerp(start[i], target[i], k);

            ApplyJointAngles(pose);
            yield return null;
        }

        ApplyJointAngles(target);
        moveRoutine = null;
    }

    // ============================================================
    // CINEMÁTICA INVERSA (CCD) – POSICIÓN
    // ============================================================

    /// <summary>
    /// Aplica CCD IK sobre la cadena Link1..Link6 para que el
    /// end-effector se acerque a targetPos. Se asume 1 DOF por
    /// joint con los ejes:
    ///  - Link1: Y
    ///  - Link2: Z
    ///  - Link3: Z
    ///  - Link4: Z
    ///  - Link5: Y
    ///  - Link6: Z
    /// </summary>
    public void SolveIKToTarget(Vector3 targetPos, int iterations, float maxStepAngleDeg)
    {
        if (endEffector == null)
            endEffector = link6;

        if (endEffector == null)
            return;

        for (int it = 0; it < iterations; it++)
        {
            // Orden inverso: desde el último joint hacia el primero
            AdjustJointTowards(link6, Vector3.forward, targetPos, maxStepAngleDeg); // eje Z local
            AdjustJointTowards(link5, Vector3.up, targetPos, maxStepAngleDeg);      // eje Y local
            AdjustJointTowards(link4, Vector3.forward, targetPos, maxStepAngleDeg); // eje Z local
            AdjustJointTowards(link3, Vector3.forward, targetPos, maxStepAngleDeg); // eje Z local
            AdjustJointTowards(link2, Vector3.forward, targetPos, maxStepAngleDeg); // eje Z local
            AdjustJointTowards(link1, Vector3.up, targetPos, maxStepAngleDeg);      // eje Y local
        }

        UpdateCurrentFromTransforms();
    }

    /// <summary>
    /// Ajusta un joint (un solo DOF) para alinear el vector joint→EE
    /// con el vector joint→target, rotando alrededor de localAxis.
    /// </summary>
    private void AdjustJointTowards(Transform joint, Vector3 localAxis, Vector3 targetPos, float maxStepAngleDeg)
    {
        if (joint == null) return;

        Vector3 jointPos = joint.position;

        Vector3 toEnd = (endEffector.position - jointPos).normalized;
        Vector3 toTarget = (targetPos - jointPos).normalized;

        // Eje de rotación en coordenadas de mundo
        Vector3 axisWorld = joint.TransformDirection(localAxis);

        // Proyectar en el plano perpendicular al eje para evitar giros locos
        Vector3 projEnd = Vector3.ProjectOnPlane(toEnd, axisWorld);
        Vector3 projTarget = Vector3.ProjectOnPlane(toTarget, axisWorld);

        if (projEnd.sqrMagnitude < 1e-6f || projTarget.sqrMagnitude < 1e-6f)
            return;

        float angle = Vector3.SignedAngle(projEnd, projTarget, axisWorld);
        float clamped = Mathf.Clamp(angle, -maxStepAngleDeg, maxStepAngleDeg);

        joint.Rotate(axisWorld, clamped, Space.World);
    }
}
