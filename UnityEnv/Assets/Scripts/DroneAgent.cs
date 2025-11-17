using System.Linq;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

[DefaultExecutionOrder(-900)]
[RequireComponent(typeof(Rigidbody))]
public class DroneAgent : Agent
{
    //  Reasons why the episode ended, used for logging and debugging 
    enum EndReason { Success, NoImprove, HardCrash, Tilt, Timeout, Stuck }

    // ADDED FOR PORCH NAVIGATION
    [Header("Porch Waypoints (auto)")]
    [SerializeField] private Transform waypointsParent;   // Drag "Waypoints_porches" here
    [SerializeField] private Transform[] porchWaypoints;  // Auto-populated from children
    private int currentPorchIndex = 0;

    // Auto-populate in Editor and at runtime
    private void OnValidate()
    {
        // If not set, try to find by common name to reduce setup friction
        if (waypointsParent == null)
        {
            var go = GameObject.Find("Waypoints_porches");
            if (go) waypointsParent = go.transform;
        }
        AutoFillWaypoints();
    }

    private void AutoFillWaypoints()
    {
        if (!waypointsParent) return;

        // Get all direct/indirect children (excluding the parent), keep inactive too.
        porchWaypoints = waypointsParent
            .GetComponentsInChildren<Transform>(includeInactive: true)
            .Where(t => t != waypointsParent)
            .OrderBy(t => t.name) // predictable ordering: porch_01, porch_02, ...
            .ToArray();
    }

    // Optional: call this at runtime if you spawn agent via prefab and wire things up in code.
    public void SetWaypointsParent(Transform parent)
    {
        waypointsParent = parent;
        AutoFillWaypoints();
    }
    // END ADDED FOR PORCH NAVIGATION

    // GLOBALS
    public Transform goal;
    public DroneAutopilot autopilot;   // reference to the hover script
    public Rigidbody rb;
    private float prevDist;
    float timer;

    [Header("Clock Scaling (for simulation speed)")]
    public float ClockScaler = 2f;

    [Header("LiDAR Input")]
    public LiDARLogger lidarLogger;
    private float[] lidarVec;
    public float lidarEps = 1e-3f; // Used for LiDAR observation normalization (avoid div0)

    [Header("Episode Bounds")]
    public Vector3 startArea = new Vector3(5, 2, 5);
    public Vector3 goalArea = new Vector3(8, 2, 8);
    public float minStartY = 2f;
    public float maxStartY = 6f;

    [Header("Proximity / Avoidance")]
    public float avoidDistance = 3.0f;     // start shaping penalty inside this radius (meters)
    public float hardAvoidDistance = 1.0f; // really close = strong penalty, maybe termination
    public float closeStuckTime = 2.0f;    // seconds too-close before early terminate

    private float lastMinLidarDist = Mathf.Infinity;
    private float closeProximityTimer = 0f;

    [Header("Success / Safety")]
    public float successRadius = 3.0f;
    public float maxTiltDeg = 45f; 
    public float maxEpisodeTime = 90f;

    [Header("Action shaping/Agent smoothing")]
    public float rpScale = 0.3f;           // roll/pitch scale 
    public float climbScale = 0.25f;       // climb scale (m/s at action=1)
    public float actionSlewPerSec = 2.0f;  // how fast actions can change
    public float warmupSeconds = 0.5f;     // zero actions at episode start

    // smoothed actions (state)
    float smRoll, smPitch, smClimb, smYaw;
    float episodeT;

    // collision handling
    private bool collisionQueued = false;
    private float queuedCollisionSpeed = 0f;
    private float lastCollisionPenaltyTime = -999f;
    private float bestDist; 
    private float noImproveTimer;

    // performance tracking
    private int collisionsThisEpisode = 0;
    private bool successThisEpisode = false;
    private StatsRecorder stats;
    
    // Hyperparams for the reward section
    [Header("Shaping")]
    public float floorY = 0.5f;
    public float ceilingY = 12f;
    public float vertTargetWeight = 0.15f;
    public float horizProgressWeight = 0.3f;
    public float alignWeight = 0.01f;
    public float controlCost = 0.0015f;
    public float tinyTimePenalty = -0.005f;

    // GOAL BEACON
    [Header("Goal Beacon (runtime)")]
    public bool showGoalBeacon = true;
    public float beaconHeight = 20f;
    public float beaconRadius = 0.12f;

    GameObject _beacon;

    void EnsureBeacon()
    {
        if (!showGoalBeacon || goal == null) return;

        if (_beacon == null)
        {
            _beacon = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            _beacon.name = "GoalBeacon";
            var col = _beacon.GetComponent<Collider>();
            if (col) Destroy(col);
            var mr = _beacon.GetComponent<MeshRenderer>();
            var shader = Shader.Find("Universal Render Pipeline/Unlit")
                        ?? Shader.Find("Unlit/Color")
                        ?? Shader.Find("Legacy Shaders/Diffuse");
            mr.material = new Material(shader);
            mr.material.color = Color.yellow;
            _beacon.transform.SetParent(goal, worldPositionStays: false);
        }

        _beacon.transform.localScale = new Vector3(beaconRadius * 2f, beaconHeight * 0.5f, beaconRadius * 2f);
        _beacon.transform.localPosition = new Vector3(0f, beaconHeight * 0.5f, 0f);
        _beacon.transform.localRotation = Quaternion.identity;
    }
    // END BEACON
    
    void Awake()
    {
        if (!rb) rb = GetComponent<Rigidbody>();
        if (!autopilot) autopilot = GetComponent<DroneAutopilot>();
        if (!lidarLogger) lidarLogger = GetComponentInParent<LiDARLogger>();
        stats = Academy.Instance.StatsRecorder;

        Time.timeScale = ClockScaler;
        Time.fixedDeltaTime = 0.02f / ClockScaler;
    }

public override void OnEpisodeBegin()
{
    // Reset metrics
    collisionsThisEpisode = 0;
    successThisEpisode = false;
    closeProximityTimer = 0f;
    lastMinLidarDist = Mathf.Infinity;
    noImproveTimer = 0f;

    // Reset physics
    rb.linearVelocity = Vector3.zero;
    rb.angularVelocity = Vector3.zero;
    rb.Sleep();

    //  Randomize start pose 
    Vector3 startPos = new Vector3(
        Random.Range(-startArea.x, startArea.x),
        Random.Range(minStartY, maxStartY),
        Random.Range(-startArea.z, startArea.z)
    );
    Debug.Log($"[Agent] Spawned at Y={startPos.y:F2}, transformY(before)={transform.position.y:F2}");

    transform.position = startPos;
    transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

    Physics.SyncTransforms();

    // Arm autopilot at the new position
    autopilot.ArmForEpisode(transform.position.y);
    autopilot.tiltCmd  = Vector2.zero;
    autopilot.climbCmd = 0f;
    autopilot.yawCmd   = 0f;

    //  Goal selection (porch / fallback) 
    if (porchWaypoints != null && porchWaypoints.Length > 0)
    {
        goal.position = porchWaypoints[currentPorchIndex].position;
        currentPorchIndex = (currentPorchIndex + 1) % porchWaypoints.Length;
    }
    else
    {
        Debug.LogWarning("No porch waypoints assigned! Using default random goal.");
        Vector3 fallbackGoal = new Vector3(
            Random.Range(-goalArea.x, goalArea.x),
            Random.Range(3f, 6f),
            Random.Range(-goalArea.z, goalArea.z)
        );
        goal.position = fallbackGoal;
    }

    EnsureBeacon();

    // Now that spawn & goal are set, initialize distance-based terms
    float d0 = Vector3.Distance(transform.position, goal.position);
    prevDist = d0;
    bestDist = d0;
    noImproveTimer = 0f;

    // Altitude target at current Y (lets agent choose when to move horizontally)
    autopilot.SetTargetY(transform.position.y);
    autopilot.tiltCmd  = Vector2.zero;
    autopilot.climbCmd = 0f;

    timer = 0f;
    episodeT = 0f;
    smRoll = smPitch = smClimb = smYaw = 0f;
}


    bool printed;
public override void CollectObservations(VectorSensor sensor)
{
    if (!printed)
    {
        int len = lidarLogger && lidarLogger.latestRow != null ? lidarLogger.latestRow.Length : 0;
        Debug.Log($"[Obs] latestRow len={len}, total obs ~= {len}+ (goal dir/alt/self)");
        printed = true;
    }

    //  LiDAR: sanitize + track min distance 
    if (lidarLogger != null && lidarLogger.latestRow != null)
    {
        lidarVec = lidarLogger.latestRow;
        float maxRange = Mathf.Max(lidarLogger.maxRange, lidarEps);
        float invMax  = 1f / maxRange;

        float minD = Mathf.Infinity;

        for (int i = 0; i < lidarVec.Length; i++)
        {
            float raw = lidarVec[i];
            float norm;
            if (float.IsNaN(raw) || float.IsInfinity(raw))
            {
                Debug.LogWarning($"[LiDAR] NaN/Inf at idx {i} for agent at {transform.position}");
                raw = maxRange;
            }
            // If we are looking at the first 16 values, they are NOT LiDAR beams so should not be used for min distance calculation
            // TODO: 16 is hardcoded, should be parameterized and MUST be updated if the LiDAR config changes
            if (i < 16) {
                // normalized for observation
                norm = Mathf.Clamp01(raw * invMax);
            } else {
                // Clamp to physical bounds: 0..maxRange
                float d = Mathf.Clamp(raw, 0f, maxRange);
                if (d != 0f & d < minD) minD = d;
                // normalized for observation
                norm = Mathf.Clamp01(d * invMax);
            }
            // Add to observation vector no matter what
            sensor.AddObservation(norm);
        }
        // Remember min distance for reward shaping
        lastMinLidarDist = minD;
    }
    else
    {
        int n = (lidarVec != null) ? lidarVec.Length : 0;
        for (int i = 0; i < n; i++) sensor.AddObservation(0f);
        lastMinLidarDist = Mathf.Infinity;
    }

    //  Goal cues (as you had) 
    Vector3 toGoal = goal.position - transform.position;
    Vector2 toGoalXZ = new Vector2(toGoal.x, toGoal.z);
    float horizDist = toGoalXZ.magnitude;
    float vertDelta = toGoal.y;

    Vector3 fwdXZ = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
    Vector3 dirXZ = Vector3.ProjectOnPlane(toGoal,            Vector3.up).normalized;

    float headingErr = Mathf.Atan2(
        Vector3.Cross(fwdXZ, dirXZ).y,
        Vector3.Dot(fwdXZ,   dirXZ));

    sensor.AddObservation(Mathf.Sin(headingErr));
    sensor.AddObservation(Mathf.Cos(headingErr));

    sensor.AddObservation(Mathf.Clamp(horizDist / 10f, 0f, 1f));
    Vector3 dirLocal = transform.InverseTransformDirection(new Vector3(toGoal.x, 0f, toGoal.z).normalized);
    sensor.AddObservation(dirLocal.x);
    sensor.AddObservation(dirLocal.z);
    sensor.AddObservation(Mathf.Clamp(vertDelta / 10f, -1f, 1f));

    // SELF-STATE OBSERVATIONS 
    // Local linear velocity (roughly in [-1,1] for speeds <= 10 m/s)
    Vector3 velLocal = transform.InverseTransformDirection(rb.linearVelocity);
    sensor.AddObservation(velLocal / 10f);

    // Local angular velocity (roughly in [-1,1])
    Vector3 angVelLocal = transform.InverseTransformDirection(rb.angularVelocity);
    sensor.AddObservation(angVelLocal / 5f);

    // Orientation relative to world up
    Vector3 up = transform.up; // components in [-1,1]
    sensor.AddObservation(up);
}

// 4 continuous actions: roll, pitch, climb, yaw
public override void OnActionReceived(ActionBuffers actions)
{
    var act = actions.ContinuousActions;
    if (act.Length != 4)
    {
        Debug.LogError($"Expected 4 continuous actions, got {act.Length}.");
        return;
    }

    float tilt = Vector3.Angle(transform.up, Vector3.up);
    timer += Time.fixedDeltaTime;
    episodeT += Time.fixedDeltaTime;

    float targetRoll  = Mathf.Clamp(act[0], -1f, 1f) * rpScale;
    float targetPitch = Mathf.Clamp(act[1], -1f, 1f) * rpScale;
    float targetClimb = Mathf.Clamp(act[2], -1f, 1f) * climbScale;
    float targetYaw   = Mathf.Clamp(act[3], -1f, 1f) * 1.0f;

    float step = actionSlewPerSec * Time.fixedDeltaTime;
    smRoll  = Mathf.MoveTowards(smRoll,  targetRoll,  step);
    smPitch = Mathf.MoveTowards(smPitch, targetPitch, step);
    smClimb = Mathf.MoveTowards(smClimb, targetClimb, step);
    smYaw   = Mathf.MoveTowards(smYaw,   targetYaw,   step);

    if (episodeT < warmupSeconds) { smRoll = smPitch = smClimb = 0f; }

    autopilot.tiltCmd   = new Vector2(smRoll, smPitch);
    autopilot.climbCmd  = smClimb;
    autopilot.yawCmd    = smYaw;

    // REWARD SECTION 
    float r_potential = 0f, r_vel = 0f, r_heading = 0f, r_yband = 0f, r_time = 0f;

    Vector3 toGoal = goal.position - transform.position;
    float dist = toGoal.magnitude;

    // track best distance for no-improvement logic
    if (dist + 0.1f < bestDist)
    {
        bestDist = dist;
        noImproveTimer = 0f;
    }
    else
    {
        noImproveTimer += Time.fixedDeltaTime;
    }

    // YAW SECTION
    Vector3 fwdXZ = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
    Vector3 dirXZ = Vector3.ProjectOnPlane(toGoal,            Vector3.up).normalized;
    float headingErr = Mathf.Atan2(
        Vector3.Cross(fwdXZ, dirXZ).y,
        Vector3.Dot(fwdXZ,   dirXZ));
    float face = Mathf.Cos(headingErr);
    AddReward(0.01f * face);

    float yawRateCur = Vector3.Dot(rb.angularVelocity, transform.up); // rad/s
    AddReward(-0.0005f * Mathf.Abs(yawRateCur));

    // (A) Potential-based shaping (distance improvement)
    float kPot = 1.0f;
    float distDelta = prevDist - dist;
    r_potential = kPot * distDelta;
    AddReward(r_potential);
    prevDist = dist;

    // (B) Velocity toward goal
    float vToward = Vector3.Dot(rb.linearVelocity, toGoal.normalized);
    r_vel = 0.02f * vToward;
    AddReward(r_vel);

    // (C) Heading alignment (nose pointing to goal)
    float heading = Vector3.Dot(transform.forward, toGoal.normalized);
    r_heading = 0.01f * heading;
    AddReward(r_heading);

    // (D) Vertical band penalty
    float yBandCenter = goal.position.y;
    float yBandHalf   = 4.0f;
    float yErrOutside = Mathf.Max(0f, Mathf.Abs(transform.position.y - yBandCenter) - yBandHalf);
    r_yband = -0.02f * yErrOutside;
    AddReward(r_yband);

    // (E) Time + distance penalty (break the hover optimum)
    float distNorm = Mathf.Clamp01(dist / 40f); // 0 when at goal, 1 when far
    r_time = tinyTimePenalty * (0.5f + distNorm); 
    // When far, ~1.5 * tinyTimePenalty; when close, ~0.5 * tinyTimePenalty
    AddReward(r_time);

    // Small living bonus when near the goal region to encourage staying there
    if (dist < successRadius * 2f)
    {
        AddReward(0.002f);
    }

    //  PROXIMITY SHAPING 
    if (!float.IsInfinity(lastMinLidarDist))
    {
        if (lastMinLidarDist < avoidDistance)
        {
            float denom = Mathf.Max(avoidDistance - hardAvoidDistance, 0.01f);
            float prox = Mathf.Clamp01((avoidDistance - lastMinLidarDist) / denom);

            AddReward(-0.02f * prox * prox);

            if (lastMinLidarDist < hardAvoidDistance)
                closeProximityTimer += Time.fixedDeltaTime;
            else
                closeProximityTimer = 0f;
        }
        else
        {
            closeProximityTimer = 0f;
        }
    }
    else
    {
        closeProximityTimer = 0f;
    }

    //  EPISODE TERMINATION CONDITIONS WITH LOGS 

    // Stuck too close to obstacle
    if (closeProximityTimer > closeStuckTime)
    {
        AddReward(-2.0f);
        RecordStats();
        Debug.Log(
            $"Episode end: {EndReason.Stuck} " +
            $"(minLiDAR={lastMinLidarDist:F2} m, " +
            $"pos={transform.position}, v={rb.linearVelocity.magnitude:F2})"
        );
        EndEpisode();
        return;
    }

    // Collision handling (unchanged)
    var impactScale = -0.02f;
    var collisionCooldown = 0.5f; 
    if (collisionQueued && (Time.time - lastCollisionPenaltyTime) >= collisionCooldown)
    {
        float penalty = -2.0f + (impactScale * queuedCollisionSpeed);
        AddReward(penalty);

        if (queuedCollisionSpeed >= 5.0f)
        {
            AddReward(-5.0f);
            RecordStats();
            Debug.Log(
                $"Episode end: {EndReason.HardCrash} " +
                $"(impactSpeed={queuedCollisionSpeed:F2}, pos={transform.position})"
            );
            EndEpisode();
            return;
        }

        collisionQueued = false;
        queuedCollisionSpeed = 0f;
        lastCollisionPenaltyTime = Time.time;
    }

    // Success
    if (dist < successRadius)
    {
        AddReward(+40f);
        successThisEpisode = true;
        RecordStats();
        Debug.Log(
            $"Episode end: {EndReason.Success} " +
            $"(dist={dist:F2}, pos={transform.position})"
        );
        EndEpisode();
        return;
    }

    // No improvement -> explicitly penalize "hover forever"
    if (noImproveTimer > 20f)
    {
        AddReward(-5.0f);
        RecordStats();
        Debug.Log(
            $"Episode end: {EndReason.NoImprove} " +
            $"(bestDist={bestDist:F2}, curDist={dist:F2}, pos={transform.position})"
        );
        EndEpisode();
        return;
    }

    // Excessive tilt
    if (tilt > maxTiltDeg)
    {
        AddReward(-0.5f);
        RecordStats();
        Debug.Log($"Episode end: {EndReason.Tilt}");
        EndEpisode();
        return;
    }

    // Timeout
    if (timer > maxEpisodeTime)
    {
        RecordStats();
        Debug.Log(
            $"Episode end: {EndReason.Timeout} " +
            $"(t={timer:F1}s, dist={dist:F2}, pos={transform.position})"
        );
        EndEpisode();
        return;
    }
}
    public void RegisterCrash(float impactSpeed = 0f)
    {
        collisionQueued = true;
        queuedCollisionSpeed = Mathf.Max(queuedCollisionSpeed, impactSpeed);
        collisionsThisEpisode++;
    }

    public void RegisterSuccess()
    {
        successThisEpisode = true;
        AddReward(+50.0f);
        RecordStats();
        Debug.Log($"Episode end: {EndReason.Success} (RegisterSuccess, pos={transform.position})");
        EndEpisode();
    }

    private void RecordStats()
    {
        stats.Add("Episode/Collisions", collisionsThisEpisode);
        stats.Add("Episode/Success", successThisEpisode ? 1 : 0);
        stats.Add("Episode/TotalReward", GetCumulativeReward());
    }

    void OnDrawGizmosSelected()
    {
        if (porchWaypoints == null) return;
        Gizmos.color = Color.yellow;
        foreach (var wp in porchWaypoints)
            if (wp != null) Gizmos.DrawSphere(wp.position, 0.3f);
    }
}
