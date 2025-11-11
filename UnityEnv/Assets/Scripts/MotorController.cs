using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class MotorController : MonoBehaviour
{
    [Header("Physical Model")]
    [Tooltip("Max static thrust (Newtons) a single prop can generate at full power.")]
    public float maxThrustPerMotor = 12f;

    [Tooltip("Number of motors (fixed at 6 for hex).")]
    [Range(6,6)] public int motorCount = 6;

    [Header("Gains (tune to your rig)")]
    [Tooltip("How strongly the controller uses measured linear acceleration (world) to infer thrust demand.")]
    public float accelGain = 1.0f;

    [Tooltip("How strongly tilt error (from level) contributes roll/pitch torque demand.")]
    public float tiltTorqueGain = 0.7f;

    [Tooltip("Angular velocity damping for roll/pitch (reduces oscillations).")]
    public float angVelDampRP = 0.2f;

    [Tooltip("Angular velocity damping for yaw (positive damps spin).")]
    public float angVelDampYaw = 0.15f;

    [Tooltip("Response smoothing (larger = snappier response).")]
    [Range(0f, 40f)] public float response = 12f;

    [Header("Debug (live readout)")]
    [Range(0f,1f)] public float prop1, prop2, prop3, prop4, prop5, prop6;

    Rigidbody rb;

    // internal state
    Vector3 prevVel;
    bool havePrev;
    readonly float[] strengths = new float[6];
    readonly float[] targets   = new float[6];

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        havePrev = false;
        for (int i = 0; i < 6; i++) { strengths[i] = 0f; targets[i] = 0f; }
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        if (dt <= 0f) return;

        // --- 1) Measured kinematics ---
        Vector3 vel = rb.linearVelocity;
        Vector3 aWorld = havePrev ? (vel - prevVel) / dt : Vector3.zero;
        prevVel = vel; havePrev = true;

        // Up vector of the body and world-up
        Vector3 up = transform.up;            // body "up"
        Vector3 worldUp = Vector3.up;

        // Tilt error: how far from level (project worldUp onto body axes)
        // Positive rollErr -> need roll torque to bring right-left level
        // Positive pitchErr -> need pitch torque to bring nose-tail level
        Vector3 right = transform.right;
        Vector3 fwd   = transform.forward;

        float rollErr  = Vector3.Dot(up, right);   // small-angle approx
        float pitchErr = Vector3.Dot(up, fwd);

        Vector3 angVel = rb.angularVelocity; // world space (rad/s)

        // --- 2) Net thrust demand along body-up ---
        // Baseline hover: m*g. Add the measured acceleration projected onto body-up.
        // The accelGain lets you scale how much measured accel affects thrust.
        float m = rb.mass;
        float g = Physics.gravity.magnitude;

        float aAlongUp = Vector3.Dot(aWorld, up);
        float totalThrustN = m * (g + accelGain * aAlongUp);  // Newtons needed along body-up

        // Distribute baseline equally
        float basePerMotor = totalThrustN / Mathf.Max(1f, motorCount);
        float baseNorm = basePerMotor / Mathf.Max(0.01f, maxThrustPerMotor); // -> 0..1 approx

        // --- 3) Torque demands -> per-motor mix (hex) ---
        // Simple PD for roll/pitch towards level + yaw damping
        float rollCmd  = (-tiltTorqueGain * rollErr)  + (-angVelDampRP * Vector3.Dot(angVel, right));
        float pitchCmd = (-tiltTorqueGain * pitchErr) + (-angVelDampRP * Vector3.Dot(angVel, fwd));
        float yawCmd   = (-angVelDampYaw * Vector3.Dot(angVel, up));

        // Mix across 6 motors around the circle (0..5 at 60� steps)
        for (int i = 0; i < 6; i++)
        {
            float a = Mathf.Deg2Rad * (i * 60f);
            float rollMix  = Mathf.Cos(a);
            float pitchMix = Mathf.Sin(a);
            float yawMix   = (i % 2 == 0) ? +1f : -1f; // alternate spin

            float u = baseNorm + rollCmd * rollMix + pitchCmd * pitchMix + yawCmd * yawMix;

            targets[i] = Mathf.Clamp01(u);
        }

        // --- 4) Smooth toward targets and publish
        float k = 1f - Mathf.Exp(-response * dt);
        for (int i = 0; i < 6; i++)
            strengths[i] = Mathf.Lerp(strengths[i], targets[i], k);

        // Debug readout in Inspector
        prop1 = strengths[0]; prop2 = strengths[1]; prop3 = strengths[2];
        prop4 = strengths[3]; prop5 = strengths[4]; prop6 = strengths[5];
    }

    /// <summary>Read current per-motor strengths (0..1). LiDARLogger calls this every tick.</summary>
    public float[] GetMotorStrengths()
    {
        var copy = new float[6];
        for (int i = 0; i < 6; i++) copy[i] = strengths[i];
        return copy;
    }
}


