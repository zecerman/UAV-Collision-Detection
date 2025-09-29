using UnityEngine;
using UnityEngine.Splines;   // Needed for Unity’s spline package

[RequireComponent(typeof(DroneAutopilot))]
public class DronePathFollower : MonoBehaviour
{
    [Header("Spline Path")]
    public SplineContainer path;   // assign your DronePath object here
    public float speed = 5f;       // movement speed along spline

    private float t = 0f;          // spline progress [0..1]
    private DroneAutopilot autopilot;

    void Start()
    {
        autopilot = GetComponent<DroneAutopilot>();
    }

    void Update()
    {
        if (path == null || autopilot == null) return;

        // Calculate normalized movement along spline
        float length = path.CalculateLength();
        t += (speed * Time.deltaTime) / length;

        if (t > 1f) t -= 1f; // loop around

        // Get position along spline
        Vector3 pos = path.EvaluatePosition(t);

        // Keep drone’s altitude controlled by autopilot
        autopilot.SetTargetY(pos.y);

        // Move drone in XZ plane toward spline point
        Vector3 targetXZ = new Vector3(pos.x, transform.position.y, pos.z);
        Vector3 toTarget = targetXZ - transform.position;

        autopilot.rb.AddForce(toTarget.normalized * speed, ForceMode.Acceleration);
    }
}
