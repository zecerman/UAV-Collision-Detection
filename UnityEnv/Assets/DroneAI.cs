using UnityEngine;

public class DroneAutopilot : MonoBehaviour
{
    // Init vars as required by Unity 
    [Header("Hover Target")]
    [Tooltip("Target world Y (meters). Leave 0 to capture current Y on Start.")]
    public float targetY = 5f;
    public bool lockTargetAtStart = false;
    // END
    // THIS BLOCK CONTROLS ROTOR BEHAVIOR INCLUDING PHYSICS-ACCURATE FORCE OUTPUT
    [System.Serializable]
    public class Rotor
    {
        public string name;
        public Transform transform;       // where the lift force is applied
        [HideInInspector] public float throttle; // 0..1 (used only for visual spinners)
    }

    public Rigidbody rb;
    public Rotor[] rotors = new Rotor[4];

    [Header("Altitude PID (outputs Newtons total)")]
    public float kp = 40f;        // proportional 
    public float ki = 5f;         // integral
    public float kd = 25f;        // derivative
    public float integralClamp = 100f; // limit windup speed

    [Header("Limits & Filtering")]
    public float maxExtraLift = 400f; // absolute limit on extra Newtons (total, not per rotor)
    public float throttleSlewPerSec = 5f; // smooth visual throttle
    // END
    // THIS BLOCK CONTROLS DRONE BALANCING  
    [Header("Level Assist (same as before)")]
    public float levelAssist = 2.0f;       // torque back to upright
    public float angularDamping = 0.25f;   // spin damping

    [Header("Altitude Stabilizer (roll/pitch PD)")]
    public float tiltKp = 40f;      // torque per radian of tilt
    public float tiltKd = 8f;       // damping against roll/pitch angular velocity
    public float yawDamp = 2f;      // damping only around up-axis
    public float maxLevelTorque = 200f; // clamp on force drone can exert 

    float integ;         // integral term
    float prevError;     // for derivative
    bool started;        // for Unity's reference

    void Reset()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Start()
    {
        rb.centerOfMass += new Vector3(0f, -0.05f, 0f); // TODO: PLACEHOLDER ADJUSTMENT TO MANUALLY ADJUST CENTER OF MASS DOWN 
        if (lockTargetAtStart || Mathf.Approximately(targetY, 0f))
            targetY = rb.position.y;   // capture current altitude
        started = true;
    }

    void FixedUpdate()
    {
        if (!rb || rotors == null || rotors.Length == 0 || !started) return;

        float dt = Time.fixedDeltaTime;
        float g = Physics.gravity.magnitude;

        //  1) Altitude PID (controls total thrust around weight) 
        float y = rb.position.y;
        float error = targetY - y;

        // integral with clamp
        integ = Mathf.Clamp(integ + error * dt, -integralClamp, integralClamp);

        // derivative (on measurement, using error difference)
        float deriv = (error - prevError) / dt;
        prevError = error;

        // PID output is "extra" Newtons (positive = more lift)
        float extraN = kp * error + ki * integ + kd * deriv;
        extraN = Mathf.Clamp(extraN, -maxExtraLift, maxExtraLift);

        // Base hover force equals weight. Distribute total across rotors.
        float totalHover = rb.mass * g;
        float totalDesired = totalHover + extraN;
        float perRotor = Mathf.Max(0f, totalDesired / rotors.Length); // N per rotor

        //  2) Apply lift at each rotor position (keeps tilt effects realistic) 
        foreach (var r in rotors)
        {
            // Smooth a 0..1 "throttle" for visuals (optional)
            float targetThrottle = Mathf.InverseLerp(0f, totalHover / rotors.Length + Mathf.Abs(maxExtraLift), perRotor);
            r.throttle = Mathf.MoveTowards(r.throttle, targetThrottle, throttleSlewPerSec * dt);

            rb.AddForceAtPosition(transform.up * perRotor, r.transform.position, ForceMode.Force);
        }

        //  3) Auto-level torque Robust roll/pitch stabilizer + yaw damping
        Vector3 up = transform.up;

        // Roll/Pitch error as axis-angle from current up to world up:
        Vector3 axis = Vector3.Cross(up, Vector3.up);             // axis to rotate around to get upright
        float sinAngle = axis.magnitude;                           // |sin(theta)|
        float angle = Mathf.Asin(Mathf.Clamp(sinAngle, 0f, 1f));   // radians in [0, pi/2] for hover use
        Vector3 tiltAxis = (sinAngle > 1e-5f) ? (axis / sinAngle) : Vector3.zero;

        // Dampen only roll/pitch angular velocity (remove yaw component)
        Vector3 w = rb.angularVelocity;
        Vector3 yawAxis = Vector3.up;                              // world-up yaw reference
        Vector3 wYaw   = Vector3.Project(w, yawAxis);
        Vector3 wRP    = w - wYaw;

        // PD torque for roll/pitch + separate yaw damping
        Vector3 torqueRP = tiltAxis * (tiltKp * angle) - wRP * tiltKd;
        Vector3 torqueYaw = -wYaw * yawDamp;

        // Clamp for safety
        Vector3 torque = Vector3.ClampMagnitude(torqueRP, maxLevelTorque) + Vector3.ClampMagnitude(torqueYaw, maxLevelTorque);

        // Apply
        rb.AddTorque(torque, ForceMode.Acceleration);
    }

    // TODO: update y value target using external triggers
    public void SetTargetY(float newY)
    {
        targetY = newY;
        // Optional: reset integral to avoid bump
        integ = 0f;
        prevError = 0f;
    }
}
