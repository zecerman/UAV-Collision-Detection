using UnityEngine;

public class EnvironmentWind : MonoBehaviour
{
    [Header("Wind Settings")]
    // the main direction the wind is blowing (X = east/west, Y = up/down, Z = forward/back)
    // public Vector3 windDirection = new Vector3(1f, 0f, 0f);
    // the base strength of the wind
    // public float windStrength = 2f;
    // the extra random strength added by gusts of wind
    // public float gustStrength = 1f;
    // how often gusts occur (higher = more frequent)
    // public float gustFrequency = 0.5f;
    [Tooltip("Base wind strength (before randomness)")]
    public float baseWindStrength = 2f;
    [Tooltip("Random variation added to base strength")]
    public float randomWindStrengthRange = 2f;

    [Tooltip("Extra random strength added by gusts of wind")]
    public float gustStrength = 1f;
    [Tooltip("How often gusts occur (higher = more frequent)")]
    public float gustFrequency = 0.5f;

    [Header("Target")]
    // the drone or object the wind will affect
    public Rigidbody droneRigidbody;
    // random time offset so gusts are not all the same for every wind object
    // private float timeOffset;
    // runtime Info (visible in Inspector but not editable)
    [Header("Current Wind (Runtime Info)")]
    [SerializeField, ReadOnlyInspector] private Vector3 windDirection;
    [SerializeField, ReadOnlyInspector] private float windStrength;

    // private Vector3 windDirection;
    // private float windStrength;
    private float timeOffset;

    // called once at the start
    void Start()
    {
        // give each wind object a random start point in the gust pattern
        timeOffset = Random.Range(0f, 10f);

        // randomize wind direction
        // Generate a random 3D direction, but bias it toward horizontal plane
        Vector2 randomDir2D = Random.insideUnitCircle.normalized; 
        windDirection = new Vector3(randomDir2D.x, Random.Range(-0.1f, 0.1f), randomDir2D.y).normalized;

        // randomize wind strength
        windStrength = baseWindStrength + Random.Range(-randomWindStrengthRange, randomWindStrengthRange);

        // Optional: log for debugging
        // Debug.Log($"New wind: direction={windDirection}, strength={windStrength:F2}");
    }
    
    // called every physics update
    void FixedUpdate()
    {
        // if no drone is assigned, do nothing
        if (droneRigidbody == null) return;

        // calculate gust using Perlin noise for smooth randomness
        float gust = Mathf.PerlinNoise(Time.time * gustFrequency + timeOffset, 0f) * gustStrength;
        // combine the main wind and the gust to get total wind force
        Vector3 totalWind = windDirection.normalized * (windStrength + gust);

        // apply the wind force to the drone's rigidbody - continuous force
        droneRigidbody.AddForce(totalWind, ForceMode.Force);
    }
}