using UnityEngine;

public class DroneAutopilot : MonoBehaviour
{
    // THIS BLOCK INIT VARS AS EXPECTED BY UNITY
    [Header("Hover Target")]
    public float targetY = 5f; // value is arbitrary until SetTargetY is called
    public float prevY;   // Bucket to assist with math which mutates targetY
    public bool lockTargetAtStart = false;
    // END
    // THIS BLOCK ALLOWS FOR MANUAL USER CONTROL OF DRONE, TODO: REMOVE BEFORE FINAL BUILD
    [Header("Manual Altitude Step Mode")]
    public bool stepMode = true;
    public float stepMeters = 0.25f;
    public float minY = 0.2f, maxY = 30f;
    public KeyCode upKey = KeyCode.UpArrow;
    public KeyCode downKey = KeyCode.DownArrow;
    // END
    // THIS BLOCK CONTROLS ROTOR BEHAVIOR INCLUDING PHYSICS-ACCURATE FORCE OUTPUT
    [System.Serializable]
    public class Rotor
    {
        public string name;
        public Transform transform;       // where the lift force is applied
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
    [Header("Level Assist")]
    public float levelAssist = 2.0f;       // torque back to upright
    public float angularDamping = 0.25f;   // spin damping

    [Header("Altitude Stabilizer")]
    // TODO: ALL THESE VALUES ARE PLACEHOLDER AND NEED TO BE TUNED FOR A QUAD WITH 4 ROTORS
    public float tiltKp = 40f;      // torque per radian of tilt
    public float tiltKd = 8f;       // damping against roll/pitch angular velocity
    public float yawDamp = 2f;      // damping only around up-axis
    public float maxLevelTorque = 200f; // clamp on force drone can exert 
    // END
    // THIS BLOCK EXPOSES 2 CONTROLS TO THE RL AGENT
    public Vector2 tiltCmd;   // x=roll [-1..1], y=pitch [-1..1]
    public float climbCmd;    // [-1..1] meters/second (or meters per update)

    [Header("RL Steering")]
    public float maxTiltBiasDeg = 10f;   // how far RL is allowed to lean TODO
    public float climbRate = 1.0f;       // m/s change to targetY per 1.0 climbCmd
    //END
    // DRIVER CODE BEGINS HERE
    float integ;         // integral term
    bool started;        // for Unity's reference

    void Reset()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Start()
    {
        rb.centerOfMass += new Vector3(0f, -0.05f, 0f); // TODO: THIS IS A PLACEHOLDER ADJUSTMENT TO MANUALLY ADJUST CENTER OF MASS DOWN 
        // Capture current altitude at start
        targetY = rb.position.y;
        prevY = rb.position.y;
        started = true;
    }
    // Human controls TODO: REMOVE BEFORE FINAL BUILD
    void Update()
    {
        if (!stepMode) return;

        if (Input.GetKeyDown(upKey))   SetTargetY(Mathf.Min(targetY + stepMeters, maxY));
        if (Input.GetKeyDown(downKey)) SetTargetY(Mathf.Max(targetY - stepMeters, minY));
    }
    // RL controls, used by external script
    void FixedUpdate()
    {
        // Safety checks
        if (!rb || rotors == null || rotors.Length == 0 || !started) return;

        // Physics constants
        float dt = Time.fixedDeltaTime;
        float g = Physics.gravity.magnitude;

        // RL: Allow RL to nudge the altitude target smoothly during episodes
        targetY += Mathf.Clamp(climbCmd, -1f, 1f) * climbRate * dt;

        // From this point, Fixed update is organized into discrete sections as denoted by 1), 2), 3), ...
        // 1) Altitude PID (controls total thrust around weight) 
        float y = rb.position.y;
        float error = targetY - y;

        // Integral with clamp
        integ = Mathf.Clamp(integ + error * dt, -integralClamp, integralClamp);

        // Derivative (calculated using "derivitive-on-measurement" using error difference to avoid kick, possible because of global prevY var)
        float deriv = -(y - prevY) / dt;
        prevY = y;

        // PID output is "extra" Newtons (positive = more lift)
        float extraN = kp * error + ki * integ + kd * deriv;
        extraN = Mathf.Clamp(extraN, -maxExtraLift, maxExtraLift);

        // Base hover force equals weight. Distribute total across rotors.
        float totalHover = rb.mass * g;
        float totalDesired = totalHover + extraN;
        float perRotor = Mathf.Max(0f, totalDesired / rotors.Length); // N per rotor

        // 2) Apply lift at each rotor position (keeps tilt effects realistic) 
        foreach (var r in rotors)
        {
            rb.AddForceAtPosition(transform.up * perRotor, r.transform.position, ForceMode.Force);
        }

        // 3) Auto-level torque Robust roll/pitch stabilizer + yaw damping
        Vector3 up = transform.up;

        // RL: Build a "desired upright" biased by RL tiltCmd
        float rollDeg = Mathf.Clamp(tiltCmd.x, -1f, 1f) * maxTiltBiasDeg;  // roll: left/right
        float pitchDeg = Mathf.Clamp(tiltCmd.y, -1f, 1f) * maxTiltBiasDeg;  // pitch: fwd/back

        // tilt around local axes: pitch about right (x), roll about forward (z)
        Quaternion desiredTilt =
            Quaternion.AngleAxis(pitchDeg, transform.right) *
            Quaternion.AngleAxis(-rollDeg, transform.forward);

        Vector3 desiredUp = desiredTilt * Vector3.up;

        // RL: Compare current 'up' to desiredUp (does not use global 'up')
        Vector3 axis = Vector3.Cross(up, desiredUp);             // axis to rotate around to get to desired up
        float sinAngle = axis.magnitude;                         // |sin(theta)|
        float angle = Mathf.Asin(Mathf.Clamp(sinAngle, 0f, 1f)); // radians in [0, pi/2] for hover use
        Vector3 tiltAxis = (sinAngle > 1e-5f) ? (axis / sinAngle) : Vector3.zero;

        // Dampen angular velocity of roll/pitch
        Vector3 w = rb.angularVelocity;
        // Dampen yaw about body-aligned axis
        Vector3 bodyUp = transform.up;
        Vector3 wYaw = Vector3.Project(w, bodyUp);

        // PD torque for roll/pitch + separate yaw damping
        Vector3 wRP = w - wYaw;     // remove yaw component
        Vector3 torqueRP = tiltAxis * (tiltKp * angle) - wRP * tiltKd;
        float yawDampEff = yawDamp * Mathf.Clamp01(Mathf.Cos(angle) + 0.25f); // less yaw damping when drone tilted (allows the drone to "carve" turns)
        Vector3 torqueYaw = -wYaw * yawDampEff;

        // Clamp (both)
        Vector3 torque = Vector3.ClampMagnitude(torqueRP, maxLevelTorque) + Vector3.ClampMagnitude(torqueYaw, maxLevelTorque);

        // Apply
        rb.AddTorque(torque, ForceMode.Acceleration);
    }
    // END
    // THIS BLOCK CONTROLS ALTITUDE TARGETING
    // RL: expose a control to change targetY and reset the drone's internal state for the altitude variables
    public void SetTargetY(float newY)
    {
        targetY = newY;
        integ = 0f;
        prevY = newY;
    }
    // END
}
