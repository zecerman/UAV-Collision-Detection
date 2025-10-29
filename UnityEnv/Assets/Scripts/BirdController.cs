using UnityEngine;

public class BirdController : MonoBehaviour
{
    [System.Serializable]
    public class Bird
    {
        public Transform birdObject;
        public float speed = 3f;              // units/sec
        public float turnSpeed = 3f;          // how fast to rotate toward desired dir
        public float changeTargetInterval = 5f;

        [HideInInspector] public Vector3 targetPosition;
        [HideInInspector] public float timer;
        [HideInInspector] public Vector3 currentVelocity; // actual movement velocity
    }

    public Bird[] birds;

    [Header("Flight Area")]
    public Vector3 areaSize = new Vector3(50f, 20f, 50f);
    public Vector3 areaCenter = Vector3.zero;

    [Header("Ground Avoidance")]
    public LayerMask groundMask;          
    public float floorClearance = 1.5f;
    public float minCruise = 2f;
    public float maxCruise = 8f;
    public float raycastHeight = 100f;

    [Header("Obstacle Avoidance")]
    public LayerMask obstacleMask;        
    public float lookAhead = 5f;          // how far we probe forward
    public float probeRadius = 0.6f;      // “thickness” of the feeler
    public float sideProbeAngle = 25f;    // degrees left/right
    public float sideProbeScale = 0.8f;   // side feelers are shorter
    public float avoidStrength = 2.0f;    // how strongly to steer away
    public float maxSteerPerSec = 6f;     // limits sudden direction changes

    void Start()
    {
        foreach (var b in birds)
        {
            KeepAboveGround(b.birdObject, true);
            SetNewTarget(b);
        }
    }

    void Update()
    {
        foreach (var b in birds)
        {
            Vector3 pos = b.birdObject.position;

            // Desired direction toward target
            Vector3 toTarget = (b.targetPosition - pos);
            Vector3 desiredDir = toTarget.sqrMagnitude > 1e-4f ? toTarget.normalized : b.birdObject.forward;

            // Compute avoidance direction (0 if clear)
            Vector3 avoid = ComputeAvoidance(pos, desiredDir);

            // Blend: target seeking + obstacle avoidance
            Vector3 blendedDir = (desiredDir + avoidStrength * avoid).normalized;

            // Optional smoothing: don’t snap direction instantly
            Vector3 newDir = Vector3.RotateTowards(
                b.currentVelocity.sqrMagnitude > 1e-6f ? b.currentVelocity.normalized : b.birdObject.forward,
                blendedDir,
                maxSteerPerSec * Time.deltaTime,
                999f
            );

            // Update velocity and position
            b.currentVelocity = newDir * b.speed;
            pos += b.currentVelocity * Time.deltaTime;

            // Stay above ground & inside vertical bounds
            KeepPositionAboveGround(ref pos, ref b.currentVelocity);

            b.birdObject.position = pos;

            // Rotate to face motion
            if (b.currentVelocity.sqrMagnitude > 1e-4f)
            {
                Quaternion targetRot = Quaternion.LookRotation(b.currentVelocity.normalized, Vector3.up);
                b.birdObject.rotation = Quaternion.Slerp(b.birdObject.rotation, targetRot, Time.deltaTime * b.turnSpeed);
            }

            // Retarget timer
            b.timer += Time.deltaTime;
            if (b.timer >= b.changeTargetInterval || (toTarget.magnitude < 1.0f))
            {
                SetNewTarget(b);
                b.timer = 0f;
            }
        }
    }

    // --- Target picking above ground ---
    void SetNewTarget(Bird bird)
    {
        Vector3 randomOffset = new Vector3(
            Random.Range(-areaSize.x * 0.5f, areaSize.x * 0.5f),
            0f,
            Random.Range(-areaSize.z * 0.5f, areaSize.z * 0.5f)
        );

        Vector3 xz = areaCenter + randomOffset;
        float groundY = GetGroundY(xz);
        float desiredAboveGround = Random.Range(minCruise, maxCruise);

        float yBottom = areaCenter.y - areaSize.y * 0.5f;
        float yTop = areaCenter.y + areaSize.y * 0.5f;
        float targetY = Mathf.Clamp(groundY + desiredAboveGround, yBottom + floorClearance, yTop);

        bird.targetPosition = new Vector3(xz.x, targetY, xz.z);
    }

    // --- Obstacle avoidance ---
    Vector3 ComputeAvoidance(Vector3 pos, Vector3 forward)
    {
        Vector3 bestAvoid = Vector3.zero;

        // 3 feelers: center, left, right
        bestAvoid = ProbeFeeler(pos, forward, lookAhead, probeRadius);
        if (bestAvoid != Vector3.zero) return bestAvoid;

        Vector3 leftDir = Quaternion.AngleAxis(-sideProbeAngle, Vector3.up) * forward;
        Vector3 rightDir = Quaternion.AngleAxis(sideProbeAngle, Vector3.up) * forward;

        Vector3 leftAvoid = ProbeFeeler(pos, leftDir, lookAhead * sideProbeScale, probeRadius * 0.9f);
        if (leftAvoid != Vector3.zero) return leftAvoid;

        Vector3 rightAvoid = ProbeFeeler(pos, rightDir, lookAhead * sideProbeScale, probeRadius * 0.9f);
        if (rightAvoid != Vector3.zero) return rightAvoid;

        return Vector3.zero;
    }

    Vector3 ProbeFeeler(Vector3 origin, Vector3 dir, float dist, float radius)
    {
        // Spherecast forward to anticipate hits
        if (Physics.SphereCast(origin, radius, dir, out RaycastHit hit, dist, obstacleMask, QueryTriggerInteraction.Ignore))
        {
            // Steer away using hit normal; bias a bit toward sliding around the obstacle
            Vector3 away = Vector3.ProjectOnPlane(dir, hit.normal); // “slide” direction
            Vector3 push = hit.normal;                               // “push” direction
            Vector3 avoid = (push * 0.7f + away * 0.3f).normalized;
            return avoid;
        }
        return Vector3.zero;
    }

    // --- Ground helpers ---
    void KeepAboveGround(Transform t, bool snapOnlyUpwards = false)
    {
        Vector3 pos = t.position;
        Vector3 vel = Vector3.zero;
        KeepPositionAboveGround(ref pos, ref vel, snapOnlyUpwards);
        t.position = pos;
    }

    void KeepPositionAboveGround(ref Vector3 pos, ref Vector3 velocity, bool snapOnlyUpwards = false)
    {
        float groundY = GetGroundY(pos);
        float minY = groundY + floorClearance;

        float yBottom = areaCenter.y - areaSize.y * 0.5f + floorClearance;
        float yTop = areaCenter.y + areaSize.y * 0.5f;
        minY = Mathf.Max(minY, yBottom);

        if (pos.y < minY)
        {
            pos.y = minY;
            if (velocity.y < 0f) velocity.y = 0f;
        }
        else if (!snapOnlyUpwards && pos.y > yTop)
        {
            pos.y = yTop;
            if (velocity.y > 0f) velocity.y = 0f;
        }
    }

    float GetGroundY(Vector3 xzPos)
    {
        Vector3 origin = new Vector3(xzPos.x, xzPos.y + raycastHeight, xzPos.z);
        if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, raycastHeight * 2f, groundMask, QueryTriggerInteraction.Ignore))
            return hit.point.y;
        return 0f;
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireCube(areaCenter, areaSize);
    }
}
