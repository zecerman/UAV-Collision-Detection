using UnityEngine;

public class DroneAutopilot : MonoBehaviour
{
    [Header("References")]
    public Rigidbody rb;
    [System.Serializable] public class Rotor { 
        public string name; 
        public Transform transform; 
    }
    public Rotor[] rotors = new Rotor[6];

    [Header("Agent Inputs")]
    public Vector2 tiltCmd;   // [x=roll [-1..1], z=pitch [-1..1]]
    public float   climbCmd;  // [-1..1] vertical/thrust bias
    public float   yawCmd;    // [-1..1] yaw

    [Header("Hover Target (for logs/UI)")]
    [SerializeField] public float targetY = 5f;
    public float TargetY => targetY;
    public float minY = 0.2f, maxY = 30f;
    public bool  clampTargetY = true;

    // Authority knobs , tunable
    [Header("Authority (Agent vs Stabilizers)")]
    [Range(0f, 1f)] public float attitudeAuthority = 0.75f; // 0 = pure stabilizer, 1 = pure agent torque
    [Range(0f, 1f)] public float yawAuthority = 0.75f;
    [Tooltip("Fraction of weight the agent may add/remove as raw thrust bias (on top of PID).")]
    [Range(0f, 1f)] public float thrustAuthority = 0.50f;
    [Tooltip("Smoothing of commands. Lower = snappier, higher = mushier.")]
    public float cmdSlewPerSec = 6f;

    // Altitude control reworked for RL: velocity PID
    [Header("Altitude Control (velocity PID for RL)")]
    public float maxClimbRate    = 3f;   // m/s, command range for vertical speed
    public float vKp             = 3f;
    public float vKi             = 0.5f;
    public float vKd             = 1f;
    public float vIntegralLimit  = 5f;

    [Header("Thrust Smoothing")]
    public float throttleSlewPerSec = 8f;     // faster causes UAV to obey agent more
    public float maxThrustG         = 2.5f;   // max thrust as multiple of weight

    [Header("Attitude Stabilizer (gentle, autopilot only)")]
    public float levelKp = 12f;     // torque per rad toward upright (smaller than before)
    public float levelKd = 4f;      // roll/pitch damping
    public float yawDamp = 0.8f;    // yaw damping

    // TODO: Most of these are maxed out anyways, remove?
    [Header("Agent Body-Rate Authority")]
    public float maxRollRateDeg  = 120f;   // higher = more authority
    public float maxPitchRateDeg = 120f;
    public float maxYawRateDeg   = 150f;
    public float rateKp = 1.2f;
    public float rateKd = 0.02f;
    public float maxTorque = 250f;

    // Globals
    float vInt, prevVError, thrustN;
    bool started, _armedByAgent;
    Vector2 smTilt;          // Smoothing state for commands
    float smClimb, smYaw;    // Smoothing state for commands

    void Reset() { rb = GetComponent<Rigidbody>(); }

    void Start()
    {
        if (!rb) rb = GetComponent<Rigidbody>();
        targetY = rb.position.y;

        // Start at hover thrust
        thrustN = rb.mass * Physics.gravity.magnitude;
        started = true;
    }

    // Agent arms each episode (prevents running with stale inputs)
    // TODO: Possibly no longer necessary, this was used to fix an old bug related to stale inputs that may no longer exist.
    public void ArmForEpisode(float newY)
    {
        SetTargetY(newY);
        _armedByAgent = true;
    }

    public void Disarm() => _armedByAgent = false;

    void FixedUpdate()
    {
        if (!_armedByAgent || !started || !rb || rotors == null || rotors.Length == 0) return;

        float dt     = Time.fixedDeltaTime;
        float g      = Physics.gravity.magnitude;
        float weight = rb.mass * g;

        //  0) Smooth agent commands (less smoothing = more authority) 
        float step = Mathf.Max(0.0001f, cmdSlewPerSec) * dt;
        smTilt.x = Mathf.MoveTowards(smTilt.x, Mathf.Clamp(tiltCmd.x, -1f, 1f), step);
        smTilt.y = Mathf.MoveTowards(smTilt.y, Mathf.Clamp(tiltCmd.y, -1f, 1f), step);
        smClimb = Mathf.MoveTowards(smClimb, Mathf.Clamp(climbCmd, -1f, 1f), step);
        smYaw = Mathf.MoveTowards(smYaw, Mathf.Clamp(yawCmd,   -1f, 1f), step);

        //    A) Altitude control (reworked: velocity PID about vertical speed)
        // Measure vertical velocity in world space
        float vY = Vector3.Dot(rb.linearVelocity, Vector3.up);

        // Agent command -> desired vertical speed
        float vY_desired = Mathf.Clamp(smClimb, -1f, 1f) * maxClimbRate;

        // Velocity error
        float vError = vY_desired - vY;

        // PID on vertical velocity
        vInt = Mathf.Clamp(vInt + vError * dt, -vIntegralLimit, vIntegralLimit);
        float vDeriv = (vError - prevVError) / Mathf.Max(1e-4f, dt);
        prevVError = vError;

        float vCorrectionN = vKp * vError + vKi * vInt + vKd * vDeriv;

        // Base hover thrust plus correction
        float desiredThrust = weight + vCorrectionN;

        // Clamp to reasonable range (0 .. maxThrustG * weight)
        float maxThrustN = weight * maxThrustG;
        desiredThrust = Mathf.Clamp(desiredThrust, 0f, maxThrustN);

        // Slew to desired
        float tstep = throttleSlewPerSec * dt * weight;
        thrustN = Mathf.MoveTowards(thrustN, desiredThrust, tstep);

        // Apply distributed lift (along body up) at rotor positions
        float perRotor = Mathf.Max(0f, thrustN / rotors.Length);
        foreach (var r in rotors)
            rb.AddForceAtPosition(transform.up * perRotor, r.transform.position, ForceMode.Force);

        //    B) Attitude control (Agent body-rate torque blended with gentle stabilizer) 
        // Desired body rates from agent (rad/s)
        float p_des = Mathf.Deg2Rad * (smTilt.x * maxRollRateDeg);
        float q_des = Mathf.Deg2Rad * (smTilt.y * maxPitchRateDeg);
        float r_des = Mathf.Deg2Rad * (smYaw    * maxYawRateDeg);

        // Current world angular vel -> body frame
        Vector3 w_world = rb.angularVelocity;
        Vector3 w_body  = transform.InverseTransformDirection(w_world);

        // Track body rates
        Vector3 w_des_body = new Vector3(p_des, r_des, q_des); // (roll, yaw, pitch) mapping
        Vector3 e_rate = w_des_body - w_body;
        Vector3 torqueRate_body = rateKp * e_rate - rateKd * w_body;
        Vector3 torqueRate_world = transform.TransformDirection(torqueRate_body);

        // Gentle upright + damping (world space)
        Vector3 up = transform.up;
        Vector3 axis = Vector3.Cross(up, Vector3.up);
        float sinA = axis.magnitude;
        Vector3 tiltAxis = (sinA > 1e-6f) ? (axis / sinA) : Vector3.zero;
        float angle = Mathf.Asin(Mathf.Clamp(sinA, 0f, 1f)); // radians

        Vector3 wYaw_world = Vector3.Project(w_world, up);
        Vector3 wRP_world  = w_world - wYaw_world;

        Vector3 torqueLevel_world = tiltAxis * (levelKp * angle) + (-wRP_world * levelKd);
        Vector3 torqueYawDamp     = -wYaw_world * yawDamp;

        // Blend of PID and agent, agent torque dominates per the authority sliders magnitude
        Vector3 torque_world =
            torqueRate_world * attitudeAuthority
          + torqueLevel_world * (1f - attitudeAuthority)
          + torqueYawDamp     * (1f - yawAuthority);

        torque_world = Vector3.ClampMagnitude(torque_world, maxTorque);
        rb.AddTorque(torque_world, ForceMode.Acceleration);
    }

    // Public helpers 
    public void SetTargetY(float newY)
    {
        targetY = clampTargetY ? Mathf.Clamp(newY, minY, maxY) : newY;
        vInt = 0f;
        prevVError = 0f;
    }
}
