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
    public float rpScale = 0.6f;           // roll/pitch scale 
    public float climbScale = 0.5f;       // climb scale (m/s at action=1)
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
    lastProx = Mathf.Infinity;
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
        currentPorchIndex = Random.Range(0, porchWaypoints.Length);
        goal.position = porchWaypoints[currentPorchIndex].position;
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
            // TODO: 10 is hardcoded, should be parameterized and MUST be updated if the LiDAR config changes
            if (i < 10) {
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
private float lastProx = Mathf.Infinity;

public override void OnActionReceived(ActionBuffers actions)
{
    var act = actions.ContinuousActions;
    if (act.Length != 4)
    {
        Debug.LogError($"Expected 4 continuous actions, got {act.Length}.");
        return;
    }

    float dt = Time.fixedDeltaTime;
    float tilt = Vector3.Angle(transform.up, Vector3.up);
    timer     += dt;
    episodeT  += dt;

    // --- ACTION SMOOTHING / AUTOPILOT COMMANDS ---

    float targetRoll  = Mathf.Clamp(act[0], -1f, 1f) * rpScale;
    float targetPitch = Mathf.Clamp(act[1], -1f, 1f) * rpScale;
    float targetClimb = Mathf.Clamp(act[2], -1f, 1f) * climbScale;
    float targetYaw   = Mathf.Clamp(act[3], -1f, 1f) * 1.0f;

    float step = actionSlewPerSec * dt;
    smRoll  = Mathf.MoveTowards(smRoll,  targetRoll,  step);
    smPitch = Mathf.MoveTowards(smPitch, targetPitch, step);
    smClimb = Mathf.MoveTowards(smClimb, targetClimb, step);
    smYaw   = Mathf.MoveTowards(smYaw,   targetYaw,   step);

    if (episodeT < warmupSeconds)
    {
        smRoll = smPitch = smClimb = 0f;
    }

    autopilot.tiltCmd  = new Vector2(smRoll, smPitch);
    autopilot.climbCmd = smClimb;
    autopilot.yawCmd   = smYaw;

    // --- REWARD SECTION ---
    Vector3 toGoal = goal.position - transform.position;
    float   dist   = toGoal.magnitude;
    float   speed  = rb.linearVelocity.magnitude;
    float tiltDeg = Vector3.Angle(transform.up, Vector3.up);

    // 1) Step-wise distance progress (main progress signal)
    float stepDelta = prevDist - dist;   // >0 = got closer this step
    prevDist = dist;

    if (dist < bestDist)
        bestDist = dist;                 // keep bestDist for logging / debug

    // reward any step that reduces distance to goal
    if (stepDelta > 0f)
    {
        AddReward(2.0f * stepDelta);
    }


    // 2) "NoImprove" = genuinely stuck: barely changing distance AND moving very slowly
    float absDelta       = Mathf.Abs(stepDelta);
    bool tinyDistChange  = absDelta < 0.02f;  
    bool verySlow        = speed   < 0.3f; 

    if (tinyDistChange && verySlow)
        noImproveTimer += dt;
    else
        noImproveTimer = 0f;

    // 3) Velocity-toward-goal shaping (horizontal + vertical)
    {
        // Horizontal component
        Vector3 toGoalXZ = new Vector3(toGoal.x, 0f, toGoal.z);
        if (toGoalXZ.sqrMagnitude > 1e-4f)
        {
            Vector3 vXZ        = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z);
            float   vTowardXZ  = Vector3.Dot(vXZ, toGoalXZ.normalized);
            AddReward(0.004f * vTowardXZ * dt);   // stronger horizontal progress
        }

        // Vertical component: only emphasize when horizontally reasonably close
        float horizDist = new Vector2(toGoal.x, toGoal.z).magnitude;
        if (horizDist < successRadius * 3.0f)
        {
            float vY          = rb.linearVelocity.y;
            float signToGoalY = Mathf.Sign(toGoal.y); // +1 if goal above, -1 if below
            float vTowardY    = vY * signToGoalY;     // positive if moving toward goal altitude
            AddReward(0.003f * vTowardY * dt);
        }

        // Yaw-rate damping (keep as you had)
        float yawRateCur = Vector3.Dot(rb.angularVelocity, transform.up);
        AddReward(-0.0005f * Mathf.Abs(yawRateCur) * dt);
    }

    // 5) Time penalty (shorter episodes are better)
    AddReward(-0.00005f);


    // 6) Small bonus for being near goal, stable and slow (but not huge)
    if (dist < successRadius * 3f)
    {
        if (speed < 1.0f && tiltDeg < 15f)
        {
            AddReward(0.005f * dt);
        }
    }

    // 7) Proximity / obstacle shaping (with escape gradient using lastProx)
    if (!float.IsInfinity(lastMinLidarDist))
    {
        float d = lastMinLidarDist;

        // If we're inside the avoidance zone and d increases, reward that (escaping walls)
        if (d < avoidDistance && lastProx < Mathf.Infinity)
        {
            float deltaClear = d - lastProx;
            if (deltaClear > 0f)
            {
                AddReward(0.02f * deltaClear);
            }
        }
        lastProx = d;

        if (d < avoidDistance)
        {
            float denom = Mathf.Max(avoidDistance - hardAvoidDistance, 0.01f);
            float prox  = Mathf.Clamp01((avoidDistance - d) / denom);

            AddReward(-0.02f * prox * prox);

            if (d < hardAvoidDistance)
                closeProximityTimer += dt;
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
        lastProx           = Mathf.Infinity;
        closeProximityTimer = 0f;
    }

    // --- EPISODE TERMINATION CONDITIONS WITH LOGS ---

    // Stuck too close to obstacle
    if (closeProximityTimer > closeStuckTime)
    {
        AddReward(-2.0f);
        RecordStats(EndReason.Stuck);
        Debug.Log(
            $"Episode end: {EndReason.Stuck} " +
            $"(minLiDAR={lastMinLidarDist:F2} m, " +
            $"pos={transform.position}, v={rb.linearVelocity.magnitude:F2})"
        );
        EndEpisode();
        return;
    }

    if (collisionQueued)
    {
        float v = queuedCollisionSpeed;

        // Strong penalty, scaled by impact speed (quadratic in v)
        float crashPenalty = -10.0f * v;
        AddReward(crashPenalty);

        RecordStats(EndReason.HardCrash);
        Debug.Log(
            $"Episode end: {EndReason.HardCrash} " +
            $"(impactSpeed={v:F2}, penalty={crashPenalty:F1}, pos={transform.position})"
        );

        // Reset collision state
        collisionQueued        = false;
        queuedCollisionSpeed   = 0f;
        lastCollisionPenaltyTime = Time.time;

        EndEpisode();
        return;
    }
    // Success
    if (dist < successRadius)
    {
        AddReward(+100f);
        successThisEpisode = true;
        RecordStats(EndReason.Success);
        Debug.Log(
            $"Episode end: {EndReason.Success} " +
            $"(dist={dist:F2}, pos={transform.position})"
        );
        EndEpisode();
        return;
    }

    // No improvement -> explicitly penalize "hover forever"
    if (noImproveTimer > 10f)
    {
        AddReward(-3.0f);
        RecordStats(EndReason.NoImprove);
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
        AddReward(-25f);
        RecordStats(EndReason.Tilt);
        Debug.Log($"Episode end: {EndReason.Tilt}");
        EndEpisode();
        return;
    }

    // Timeout
    if (timer > maxEpisodeTime)
    {
        RecordStats(EndReason.Timeout);
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
    // Mark that a collision happened this episode and track the worst impact speed
    collisionQueued = true;
    queuedCollisionSpeed = Mathf.Max(queuedCollisionSpeed, impactSpeed);
    collisionsThisEpisode++;
}

public void RegisterSuccess()
{
    successThisEpisode = true;
    AddReward(+100.0f);
    RecordStats(EndReason.Success);
    Debug.Log($"Episode end: {EndReason.Success} (RegisterSuccess, pos={transform.position})");
    EndEpisode();
}

private void RecordStats(EndReason reason)
{
    // Episode stats
    stats.Add("Episode/TotalReward", GetCumulativeReward());

    // Explicit logs for each end reason (so they’re not all mashed into 0)
    stats.Add("Episode/Ended/Success",   reason == EndReason.Success   ? 1 : 0);
    stats.Add("Episode/Ended/HardCrash", reason == EndReason.HardCrash ? 1 : 0);
    stats.Add("Episode/Ended/Timeout",   reason == EndReason.Timeout   ? 1 : 0);
    stats.Add("Episode/Ended/NoImprove", reason == EndReason.NoImprove ? 1 : 0);
    stats.Add("Episode/Ended/Stuck",     reason == EndReason.Stuck     ? 1 : 0);
    stats.Add("Episode/Ended/Tilt",      reason == EndReason.Tilt      ? 1 : 0);

    // Convenience: outcome = 1 only for success, (this was the legacy metric)
    int outcome = (reason == EndReason.Success) ? 1 : 0;
    stats.Add("Episode/Outcome", outcome);
}

}
