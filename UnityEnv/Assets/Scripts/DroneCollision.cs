using UnityEngine;

// Detects collisions for the drone and reports them to the DroneAgent.
// Attach this to the root drone GameObject (the same one that has DroneAgent).
[RequireComponent(typeof(Collider))]
public class DroneCollision : MonoBehaviour
{
    private DroneAgent agent;

    void Awake()
    {
        agent = GetComponent<DroneAgent>();
        if (agent == null)
            Debug.LogWarning("DroneCollision: No DroneAgent found on this GameObject!");
    }

    void OnCollisionEnter(Collision collision)
    {
        // Ignore triggers (like sensors) and small bumps
        if (collision.collider.isTrigger) return;

        // Check for environment layers or tags to ignore
        string tag = collision.collider.tag.ToLower();

        // Crash if we hit anything that's not the goal
        if (tag != "goal")
        {
            float impact = collision.relativeVelocity.magnitude;
            float penalty = Mathf.Clamp(-impact * 0.05f, -5f, -0.2f);
            agent.AddReward(penalty);
            agent.RegisterCrash();

            // Optional: visual feedback
            Debug.Log($"Drone collided with {tag} (impact {impact:F2}) — penalty {penalty:F2}");
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
            agent.AddReward(+10f);
            agent.RegisterSuccess();
        }
    }
}
