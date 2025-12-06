// test cinematica inversa
using UnityEngine;
using System.Collections;

public class RobotJointController : MonoBehaviour
{
    [Header("UR3 Links (autodetect)")]
    public Transform link1, link2, link3, link4, link5, link6;

    [Header("End Effector (opcional, por defecto Link6)")]
    public Transform endEffector;

    private float[] current = new float[6];
    private Coroutine moveRoutine = null;

    // ============================================================
    // AUTO-DETECCIÓN DE JERARQUÍA DEL UR3
    // ============================================================
    public void AutoDetectJoints(Transform robotRoot)
    {
        //Transform baseNode = robotRoot.Find("UR3_Robot_model/Base");
        Transform baseNode = robotRoot.Find("base_link");

        link1 = baseNode.Find("shoulder_link");
        link2 = link1.Find("upper_arm_link");
        link3 = link2.Find("forearm_link");
        link4 = link3.Find("wrist_1_link");
        link5 = link4.Find("wrist_2_link");
        link6 = link5.Find("wrist_3_link");
        if (endEffector == null)
            endEffector = link6;

        Debug.Log("UR3 joints autodetect OK.");
        Debug.Log($"joints: {link1} {link2} {link3} {link4} {link5} {link6} {endEffector}");
        //UpdateCurrentFromTransforms();
    }

    // ============================================================
    // APLICAR ÁNGULOS A LOS JOINTS (MODO JOINT SPACE)
    // ============================================================
    public void ApplyJointAngles(float[] q) //q en radianes
    {
        if (q == null || q.Length < 6)
        {
            Debug.LogWarning("[RobotJointController] q inválido");
            return;
        }

        // Guardar estado actual
        for (int i = 0; i < 6; i++)
            current[i] = q[i];

        Debug.Log($"ApplyJointAngles: q{q} current: {current}");
        Debug.Log($"joints: {link1} {link2} {link3} {link4} {link5} {link6}");

        //joint_fixed_pose
        //Conversion a grados - calibracion sentido ejes
        float q0 = -q[0] * Mathf.Rad2Deg;
        float q1 = (q[1] + 1.57f) * Mathf.Rad2Deg;
        float q2 = q[2] * Mathf.Rad2Deg;
        float q3 = (q[3] + 1.57f) * Mathf.Rad2Deg;
        float q4 = q[4] * Mathf.Rad2Deg;
        float q5 = q[5] * Mathf.Rad2Deg;
        Debug.Log($"grados: {q0}, {q1}, {q2}, {q3}, {q4}, {q5}");

        link1.localRotation = Quaternion.Euler(0, 0, q0);  // Joint1 → eje Z
        link2.localRotation = Quaternion.Euler(0, q1, 0);  // Joint2 → eje Y
        link3.localRotation = Quaternion.Euler(0, q2, 0);  // Joint3 → eje Y
        link4.localRotation = Quaternion.Euler(0, q3, 0);  // Joint4 → eje Y
        link5.localRotation = Quaternion.Euler(0, 0, q4);  // Joint5 → eje Z
        link6.localRotation = Quaternion.Euler(0, q5, 0);  // Joint6 → eje Y
    }

    // PRUEBA DE MOVIMIENTO DE EJES UR DIGITAL
    public void SetJoint(int index, float angleDeg)
    {
        float angleRad = angleDeg * Mathf.Deg2Rad;
        Debug.Log($"angleRad {angleRad}");

        switch (index)
        {
            case 0: link1.localRotation = Quaternion.Euler(0, 0, angleDeg); break;  // Joint1 → z
            case 1: link2.localRotation = Quaternion.Euler(0, angleDeg, 0); break;  // Joint2 → y
            case 2: link3.localRotation = Quaternion.Euler(0, angleDeg, 0); break;  // Joint3 → y
            case 3: link4.localRotation = Quaternion.Euler( 0, angleDeg, 0); break;  // Joint4 → y
            case 4: link5.localRotation = Quaternion.Euler(0, 0, angleDeg); break;  // Joint5 → z
            case 5: link6.localRotation = Quaternion.Euler(0, angleDeg, 0); break;  // Joint6 → y
        }
    }

    public IEnumerator TestJointsRoutine()
    {
        float testAngle = 30f;

        for (int j = 0; j < 6; j++)
        {
            Debug.Log($"=== Probando Joint {j + 1} ===");

            // +30°
            SetJoint(j, testAngle);
            yield return new WaitForSeconds(2f);

            // -30°
            SetJoint(j, -testAngle);
            yield return new WaitForSeconds(2f);

            // Regresar
            SetJoint(j, 0f);
            yield return new WaitForSeconds(2f);
        }

        Debug.Log("===== FIN TEST JOINTS UNITY =====");
    }



    public float[] GetCurrentAngles() => current;

    //private void UpdateCurrentFromTransforms()
    //{
    //    if (link1) current[0] = NormalizeAngle(link1.localEulerAngles.y);
    //    if (link2) current[1] = NormalizeAngle(link2.localEulerAngles.z);
    //    if (link3) current[2] = NormalizeAngle(link3.localEulerAngles.z);
    //    if (link4) current[3] = NormalizeAngle(link4.localEulerAngles.z);
    //    if (link5) current[4] = NormalizeAngle(link5.localEulerAngles.y);
    //    if (link6) current[5] = NormalizeAngle(link6.localEulerAngles.z);
    //}

    //private float NormalizeAngle(float angle)
    //{
    //    angle %= 360f;
    //    if (angle > 180f) angle -= 360f;
    //    return angle;
    //}

    //// ============================================================
    //// MOVER HACIA UNA POSE EN UN TIEMPO (INTERPOLACIÓN SUAVE)
    //// ============================================================
    //public void MoveToPose(float[] target, float duration)
    //{
    //    if (moveRoutine != null)
    //        StopCoroutine(moveRoutine);

    //    moveRoutine = StartCoroutine(MoveRoutine(target, duration));
    //}

    //private IEnumerator MoveRoutine(float[] target, float duration)
    //{
    //    float[] start = GetCurrentAngles();
    //    float t = 0f;

    //    while (t < duration)
    //    {
    //        t += Time.deltaTime;
    //        float k = Mathf.Clamp01(t / duration);

    //        float[] pose = new float[6];
    //        for (int i = 0; i < 6; i++)
    //            pose[i] = Mathf.Lerp(start[i], target[i], k);

    //        ApplyJointAngles(pose);
    //        yield return null;
    //    }

    //    ApplyJointAngles(target);
    //    moveRoutine = null;
    //}

    // ============================================================
    // CINEMÁTICA INVERSA (CCD) – POSICIÓN
    // ============================================================

    ///// <summary>
    ///// Aplica CCD IK sobre la cadena Link1..Link6 para que el
    ///// end-effector se acerque a targetPos. Se asume 1 DOF por
    ///// joint con los ejes:
    /////  - Link1: Y
    /////  - Link2: Z
    /////  - Link3: Z
    /////  - Link4: Z
    /////  - Link5: Y
    /////  - Link6: Z
    ///// </summary>
    //public void SolveIKToTarget(Vector3 targetPos, int iterations, float maxStepAngleDeg)
    //{
    //    if (endEffector == null)
    //        endEffector = link6;

    //    if (endEffector == null)
    //        return;

    //    for (int it = 0; it < iterations; it++)
    //    {
    //        // Orden inverso: desde el último joint hacia el primero
    //        AdjustJointTowards(link6, Vector3.forward, targetPos, maxStepAngleDeg); // eje Z local
    //        AdjustJointTowards(link5, Vector3.up, targetPos, maxStepAngleDeg);      // eje Y local
    //        AdjustJointTowards(link4, Vector3.forward, targetPos, maxStepAngleDeg); // eje Z local
    //        AdjustJointTowards(link3, Vector3.forward, targetPos, maxStepAngleDeg); // eje Z local
    //        AdjustJointTowards(link2, Vector3.forward, targetPos, maxStepAngleDeg); // eje Z local
    //        AdjustJointTowards(link1, Vector3.up, targetPos, maxStepAngleDeg);      // eje Y local
    //    }

    //    UpdateCurrentFromTransforms();
    //}

    /// <summary>
    /// Ajusta un joint (un solo DOF) para alinear el vector joint→EE
    /// con el vector joint→target, rotando alrededor de localAxis.
    /// </summary>
    //private void AdjustJointTowards(Transform joint, Vector3 localAxis, Vector3 targetPos, float maxStepAngleDeg)
    //{
    //    if (joint == null) return;

    //    Vector3 jointPos = joint.position;

    //    Vector3 toEnd = (endEffector.position - jointPos).normalized;
    //    Vector3 toTarget = (targetPos - jointPos).normalized;

    //    // Eje de rotación en coordenadas de mundo
    //    Vector3 axisWorld = joint.TransformDirection(localAxis);

    //    // Proyectar en el plano perpendicular al eje para evitar giros locos
    //    Vector3 projEnd = Vector3.ProjectOnPlane(toEnd, axisWorld);
    //    Vector3 projTarget = Vector3.ProjectOnPlane(toTarget, axisWorld);

    //    if (projEnd.sqrMagnitude < 1e-6f || projTarget.sqrMagnitude < 1e-6f)
    //        return;

    //    float angle = Vector3.SignedAngle(projEnd, projTarget, axisWorld);
    //    float clamped = Mathf.Clamp(angle, -maxStepAngleDeg, maxStepAngleDeg);

    //    joint.Rotate(axisWorld, clamped, Space.World);
    //}
}
