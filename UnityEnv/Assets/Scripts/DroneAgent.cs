using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(Rigidbody))]
public class DroneAgent : Agent
{
    // ADDED FOR PORCH NAVIGATION
    [Header("Porch Waypoints")]
    public Transform[] porchWaypoints;  // Assign these manually in Unity
    private int currentPorchIndex = 0;
    // END ADDED FOR PORCH NAVIGATION

    public Transform goal;
    public DroneAutopilot autopilot;   // reference to the hover script
    public Rigidbody rb;

    [Header("Episode Bounds")]
    public Vector3 startArea = new Vector3(5, 2, 5); // TODO: hard coded positions are a placeholder solution
    public Vector3 goalArea = new Vector3(8, 2, 8); // TODO: hard coded positions are a placeholder solution
    public float minStartY = 2f;
    public float maxStartY = 6f;

    [Header("Success / Safety")]
    public float successRadius = 1.0f;
    public float maxTiltDeg = 45f; // TODO: TOO FAR?
    public float maxEpisodeTime = 30f;

    float timer;
    
    // TODO: thought this was necessary firve code but it has 0 references. Is it necessary? Correct?
    void Awake()
    {
        if (!rb) rb = GetComponent<Rigidbody>();
        if (!autopilot) autopilot = GetComponent<DroneAutopilot>();
    }

    public override void OnEpisodeBegin()
    {
        // Reset physics
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Randomize start
        Vector3 startPos = new Vector3(
            Random.Range(-startArea.x, startArea.x),
            Random.Range(minStartY, maxStartY),
            Random.Range(-startArea.z, startArea.z)
        );
        transform.position = startPos;
        transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

        // Randomize goal
        // REMOVED FOR PORCH NAVIGATION
        // Vector3 goalPos = new Vector3(
        //     Random.Range(-goalArea.x, goalArea.x),
        //     Random.Range(10f, 15f),     // optional: vary height target
        //     Random.Range(-goalArea.z, goalArea.z)
        // );
        // goal.position = goalPos;
        // END REMOVED FOR PORCH NAVIGATION

        // ADDED FOR PORCH NAVIGATION
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
        autopilot.SetTargetY(transform.position.y);  // Clear targetY
        autopilot.tiltCmd = Vector2.zero;                   // Clear tilt
        autopilot.climbCmd = 0f;                            // Clear climb
        timer = 0f;                                         // Reset timer
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Relative goal in drone local frame
        Vector3 rel = transform.InverseTransformPoint(goal.position);
        sensor.AddObservation(rel);                 // 3

        // Velocities in local frame
        Vector3 vLocal = transform.InverseTransformDirection(rb.linearVelocity);
        Vector3 wLocal = transform.InverseTransformDirection(rb.angularVelocity);
        sensor.AddObservation(vLocal);              // 3
        sensor.AddObservation(wLocal);              // 3

        // Altitude error and tilt
        float altErr = autopilot.targetY - transform.position.y;
        sensor.AddObservation(altErr);              // 1

        Vector3 upLocal = transform.InverseTransformDirection(transform.up);
        sensor.AddObservation(upLocal);             // 3 (tilt info)

        // Total = 13 floats (compact for now, will become more problematic when LiDAR is used)
    }

    // 3 continuous actions: roll, pitch, climb
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Saftey check, is the script configured correctly in unity?
        var act = actions.ContinuousActions;
        #if UNITY_EDITOR
        if (act.Length != 3)
        {
            Debug.LogError($"Expected 3 continuous actions but got {act.Length}. " +
                        "Check Behavior Parameters > Actions (Continuous=3, Discrete=0).");
            return;
        }
        #endif

        // Create 3 actions which the agent can use to control the drone: tiltx, tilty, and climb
        float roll = Mathf.Clamp(act[0], -1f, 1f);
        float pitch = Mathf.Clamp(act[1], -1f, 1f);
        float climb = Mathf.Clamp(act[2], -1f, 1f);
        // These ^ are the ONLY actions available to the agent

        autopilot.tiltCmd = new Vector2(roll, pitch);
        autopilot.climbCmd = climb;

        // ===REWARD SECTION===
        // Globals (resused by multiple rewards/penalties)
        float tilt = Vector3.Angle(transform.up, Vector3.up);

        // 1) Time penalty
        timer += Time.fixedDeltaTime;

        // 2) Distance travelled penalty, higher if movoing away from the goal
        float dist = Vector3.Distance(transform.position, goal.position);
        AddReward(-0.001f);                 // step cost
        AddReward(-0.001f * dist);          // push to get closer

        // 3) Success condition
        if (dist < successRadius && rb.linearVelocity.magnitude < 0.5f && tilt < 10f)
        {
            AddReward(+2.0f);
            EndEpisode();
        }

        // FAILURES, PENALIZED HARSHLY
        // 4) Excessive tilt
        if (tilt > maxTiltDeg || transform.position.y < 0.2f || timer > maxEpisodeTime)
        {
            AddReward(-1.0f);
            EndEpisode();
        }
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
