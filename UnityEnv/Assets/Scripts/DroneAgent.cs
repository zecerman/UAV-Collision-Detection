using System.Linq;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(Rigidbody))]
public class DroneAgent : Agent
{
    // --- End reasons for logging ---
    enum EndReason { Success, NoImprove, HardCrash, Tilt, Timeout }

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

    // END
    [Header("LiDAR Input")]
    public LiDARLogger lidarLogger;
    private float[] lidarVec;
    public float lidarEps = 1e-3f; // Used for LiDAR observation normalization (avoid div0)

    [Header("Episode Bounds")]
    public Vector3 startArea = new Vector3(5, 2, 5); // TODO: hard coded positions are a placeholder solution
    public Vector3 goalArea = new Vector3(8, 2, 8); // TODO: hard coded positions are a placeholder solution
    public float minStartY = 2f;
    public float maxStartY = 6f;

    [Header("Success / Safety")]
    public float successRadius = 3.0f;
    public float maxTiltDeg = 45f; // TODO: TOO FAR?
    public float maxEpisodeTime = 90f;
    // AGENT SMOTHING PARAMETERS
    [Header("Action shaping/Agent smoothing")]
    public float rpScale = 0.3f;           // roll/pitch scale (≤ 0.5 to start)
    public float climbScale = 0.5f;        // climb scale (m/s at action=1)
    public float actionSlewPerSec = 2.0f;  // how fast actions can change
    public float warmupSeconds = 1.0f;     // zero actions at episode start
    // END 
    // smoothed actions (state)
    float smRoll, smPitch, smClimb;
    float episodeT;

    // COLLISION PENALTY HANDLING
    [Header("Collision Penalties")]
    [Tooltip("Base penalty when touching an obstacle (applied once per cooldown).")]
    public float collisionPenalty = -0.2f;

    [Tooltip("Additional penalty scaled by impact speed")]
    public float impactScale = -0.02f;

    [Tooltip("If true, a 'hard crash' (floor or very strong impact) ends the episode.")]
    public bool endEpisodeOnCrash = true;

    [Tooltip("If relative speed exceeds this on, treat as hard crash.")]
    public float hardCrashSpeed = 5.0f;

    [Tooltip("Minimum time between collision penalties to prevent spam.")]
    public float collisionCooldown = 0.25f;
    // Extra globals to manage internal collision state (consumed in OnActionReceived)
    private bool collisionQueued = false;
    private float queuedCollisionSpeed = 0f;
    private float lastCollisionPenaltyTime = -999f;
    private float bestDist; private float noImproveTimer; // Related, do not uncouple
    // END

    // performance tracking
    private int collisionsThisEpisode = 0;
    private bool successThisEpisode = false;
    private StatsRecorder stats;

    // TODO: thought this was necessary dirver code but it has 0 references. Is it necessary? Correct?
    void Awake()
    {
        if (!rb) rb = GetComponent<Rigidbody>();
        if (!autopilot) autopilot = GetComponent<DroneAutopilot>();
        if (!lidarLogger) lidarLogger = GetComponentInParent<LiDARLogger>();
        stats = Academy.Instance.StatsRecorder;
    }

    public override void OnEpisodeBegin()
    {
        // Reset metrics
        collisionsThisEpisode = 0;
        successThisEpisode = false;

        // Reset physics
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        
        // Recall previous distance to goal (consumed by reward calculation)
        prevDist = Vector3.Distance(transform.position, goal.position);
        bestDist = prevDist; noImproveTimer = 0f;

        // Randomize start
        Vector3 startPos = new Vector3(
            Random.Range(-startArea.x, startArea.x),
            Random.Range(minStartY, maxStartY),
            Random.Range(-startArea.z, startArea.z)
        );
        transform.position = startPos;
        // Set random yaw, begin with upright rotation
        transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

        // ADDED FOR PORCH NAVIGATION (randomized goal)
        // Choose the next porch waypoint as the goal
        if (porchWaypoints != null && porchWaypoints.Length > 0)
        {
            goal.position = porchWaypoints[currentPorchIndex].position;

            // Cycle through porch goals each episode
            currentPorchIndex = (currentPorchIndex + 1) % porchWaypoints.Length;
        }
        else
        {
            Debug.LogWarning("No porch waypoints assigned! Using default random goal.");
            Vector3 fallbackGoal = new Vector3(
                Random.Range(-goalArea.x, goalArea.x),
                Random.Range(10f, 15f),
                Random.Range(-goalArea.z, goalArea.z)
            );
            goal.position = fallbackGoal;
        }
        // END ADDED FOR PORCH NAVIGATION

        // Cleanup autopilot's internal state at beginning of episode
        autopilot.SetTargetY(transform.position.y);         // Clear targetY
        autopilot.tiltCmd = Vector2.zero;                   // Clear tilt
        autopilot.climbCmd = 0f;                            // Clear climb
        timer = 0f;                                         // Reset timer

        // reset local action smoothing timer
        episodeT = 0f;
        smRoll = smPitch = smClimb = 0f;
    }

    bool printed;
    public override void CollectObservations(VectorSensor sensor)
    {
        // One time debug print to ensure correct observation size, etc...
        if (!printed)
        {
            int len = lidarLogger && lidarLogger.latestRow != null ? lidarLogger.latestRow.Length : 0;
            Debug.Log($"latestRow len={len}, total obs will be {len}+13");
            printed = true;
        }
        // --- MAIN OBSERVATIONS (last LiDAR row) ---
        if (lidarLogger != null && lidarLogger.latestRow != null)
        {
            lidarVec = lidarLogger.latestRow;
            float invMax = 1f / Mathf.Max(lidarLogger.maxRange, lidarEps);
            for (int i = 0; i < lidarVec.Length; i++)
            {
                float d = lidarVec[i];
                if (float.IsNaN(d) || float.IsInfinity(d)) d = lidarLogger.maxRange;   // guard
                sensor.AddObservation(Mathf.Clamp01(d * invMax));
            }
        }
        else
        {
            // Keep size consistent if missing: feed zeros
            int n = (lidarVec != null) ? lidarVec.Length : 0;
            for (int i = 0; i < n; i++) sensor.AddObservation(0f);
        }

        // EXTRA: distance + unit direction to goal (local)
        Vector3 toGoal = goal.position - transform.position;
        float distToGoal = toGoal.magnitude;
        Vector3 dirLocal = transform.InverseTransformDirection(toGoal.normalized);
        sensor.AddObservation(distToGoal);   // 1
        sensor.AddObservation(dirLocal);     // 3
    }

    // 3 continuous actions: roll, pitch, climb
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Saftey check, is the script configured correctly in unity?
        var act = actions.ContinuousActions;
        if (act.Length != 3)
        {
            Debug.LogError($"Expected 3 continuous actions, got {act.Length}. " +
                            "Check Behavior Parameters: Continuous Actions should be 3, Discrete 0, and Model empty during training.");
            return;
        }
        
        // Globals (resused by multiple rewards/penalties)
        float tilt = Vector3.Angle(transform.up, Vector3.up);
        timer += Time.fixedDeltaTime;
        episodeT += Time.fixedDeltaTime; // TODO why two timers agian?

        // Create 3 actions which the agent can use to control the drone: tiltx, tilty, and climb
        float targetRoll  = Mathf.Clamp(act[0], -1f, 1f) * rpScale;
        float targetPitch = Mathf.Clamp(act[1], -1f, 1f) * rpScale;
        float targetClimb = Mathf.Clamp(act[2], -1f, 1f) * climbScale;
        // These ^ are the ONLY actions available to the agent

        // Smooth the actions to avoid jerky commands
        float step = actionSlewPerSec * Time.fixedDeltaTime;
        smRoll  = Mathf.MoveTowards(smRoll,  targetRoll,  step);
        smPitch = Mathf.MoveTowards(smPitch, targetPitch, step);
        smClimb = Mathf.MoveTowards(smClimb, targetClimb, step);

        // Warm-up (let the hover settle before injecting control commands)
        if (episodeT < warmupSeconds) { smRoll = smPitch = smClimb = 0f; }

        // Feed autopilot actions 
        autopilot.tiltCmd = new Vector2(smRoll, smPitch); // still in [-rpScale, +rpScale]
        autopilot.climbCmd = smClimb;

        // ===REWARD SECTION===
        // REWARDS
        AddReward(-0.05f); // time penalty
        
        // Distance reward, higher if moving towards the goal but becomes negative if moving away
        float dist = Vector3.Distance(transform.position, goal.position);
        if (dist + 0.5f < bestDist) { bestDist = dist; noImproveTimer = 0f; }
        else noImproveTimer += Time.fixedDeltaTime;

        // Early stopping condition for no improvement
        if (noImproveTimer > 40f)
        {
            AddReward(-5.0f);
            RecordStats();
            Debug.Log($"Episode end: {EndReason.NoImprove}  dist={dist:F2}");
            EndEpisode();
            return;
        }

        // Always:
        AddReward(0.2f * (prevDist - dist));   // + if closer, - if farther
        prevDist = dist;

        // Alignment reward, reward for facing toward the goal, can be negative if facing away
        Vector3 dir = (goal.position - transform.position).normalized;
        float align = Vector3.Dot(transform.forward, dir);      // -1..1
        AddReward(0.02f * align);

        // Success condition
        if (dist < successRadius)// && rb.linearVelocity.magnitude < 0.5f && tilt < 10f)
        {
            AddReward(+50.0f);
            successThisEpisode = true;
            RecordStats();
            Debug.Log($"Episode end: {EndReason.Success}  dist={dist:F2}");
            EndEpisode();
        }

        // FAILURES
        // Collision penalty handling
        if (collisionQueued && (Time.time - lastCollisionPenaltyTime) >= collisionCooldown)
        {
            // Collision are always bad, punish
            float penalty = collisionPenalty + (impactScale * queuedCollisionSpeed);
            AddReward(penalty); 
            // If bad enough, a collision can end the episode
            if (endEpisodeOnCrash && (queuedCollisionSpeed >= hardCrashSpeed))
            {
                AddReward(-1.0f); // fatal crash has extra penalty
                RecordStats();
                Debug.Log($"Episode end: {EndReason.HardCrash}  speed={queuedCollisionSpeed:F2}");
                EndEpisode();
                // return;
            }

            // Consume event and start cooldown
            collisionQueued = false;
            queuedCollisionSpeed = 0f;
            lastCollisionPenaltyTime = Time.time;
        }

        // Excessive tilt or timeout
        if (tilt > maxTiltDeg || timer > maxEpisodeTime)
        {
            AddReward(-1.0f);
            RecordStats();

            if (tilt > maxTiltDeg)
                Debug.Log($"Episode end: {EndReason.Tilt}  tilt={tilt:F1}deg");
            else
                Debug.Log($"Episode end: {EndReason.Timeout}  t={timer:F1}s");

            EndEpisode();   // failure reached, should end episode
        }
    }  

    // Hooks for DroneCollision.cs
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
        Debug.Log($"Episode end: {EndReason.Success} (RegisterSuccess)");
        EndEpisode();
    }

    // Record performance metrics for TensorBoard
    private void RecordStats()
    {
        stats.Add("Episode/Collisions", collisionsThisEpisode);
        stats.Add("Episode/Success", successThisEpisode ? 1 : 0);
        stats.Add("Episode/TotalReward", GetCumulativeReward());
    }

    // ADDED FOR PORCH NAVIGATION
    // visualize porch waypoints in editor
    void OnDrawGizmosSelected()
    {
        if (porchWaypoints == null) return;
        Gizmos.color = Color.yellow;
        foreach (var wp in porchWaypoints)
        {
            if (wp != null)
                Gizmos.DrawSphere(wp.position, 0.3f);
        }
    }
    // END ADDED FOR PORCH NAVIGATION
}
