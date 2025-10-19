using UnityEngine;

public class EnvironmentWind : MonoBehaviour
{
    [Header("Wind Settings")]
    public Vector3 baseDirection = new Vector3(1, 0, 0); // base wind direction
    public float windStrength = 5f; // average wind speed
    public float gustStrength = 2f; // max gust amplitude
    public float gustFrequency = 1f; // seconds per gust update

    [Header("Targets")]
    public Rigidbody[] affectedRigidbodies; // objects affected by wind - really just the drone

    [Header("Scaling")]
    [Tooltip("How strongly the wind affects drones vs environment")]
    public float droneForceScale = 1f; // full wind force for drones
    public float envForceScale = 0.2f; // reduced force for environment objects

    // static wind info accessible by other scripts
    public static Vector3 currentWind { get; private set; } // current wind vector
    public static float currentWindStrength { get; private set; } // current wind magnitude

    // private float gustTimer;

    void Start()
    {
        // initialize wind with base direction and strength
        currentWind = baseDirection.normalized * windStrength;
        currentWindStrength = windStrength;
    }

    void FixedUpdate()
    {
        // generate smooth gust variation using Perlin noise
        float t = Time.time * 0.1f; // time scaling factor for gust speed
        float gust = (Mathf.PerlinNoise(t, 0f) - 0.5f) * 2f * gustStrength; // range: [-gustStrength, gustStrength]

        // calculate new wind strength with gust added
        float newStrength = Mathf.Max(0f, windStrength + gust); // to avoid negative wind

        // update global wind vector and strength
        currentWind = baseDirection.normalized * newStrength;
        currentWindStrength = newStrength;

        // apply wind force to each affected Rigidbody
        foreach (Rigidbody rb in affectedRigidbodies)
        {
            if (rb == null) continue; // skip null entries

            // determine force scale based on tag
            float scale = rb.CompareTag("Drone") ? droneForceScale : envForceScale;

            // apply wind force in current direction
            rb.AddForce(currentWind * scale, ForceMode.Force);
        }
    }
}