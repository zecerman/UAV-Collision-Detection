using UnityEngine;

public class WindSway : MonoBehaviour
{
    [Header("Sway Settings")]
    public float swayAmount = 5f;    // max rotation in degrees
    public float swaySpeed = 1.0f;   // speed of oscillation
    public bool useParent = true;    // rotate parent instead of mesh

    private Quaternion initialRotation;
    private Transform targetTransform;

    void Start()
    {
        // Rotate parent if desired
        targetTransform = useParent && transform.parent != null ? transform.parent : transform;
        initialRotation = targetTransform.localRotation;
    }

    void Update()
    {
        if (EnvironmentWind.currentWindStrength <= 0f) return;

        // Wind info
        Vector3 windDir = EnvironmentWind.currentWind.normalized;
        float windMag = EnvironmentWind.currentWindStrength;

        // Sway with sine + Perlin noise for gusts
        float t = Time.time * swaySpeed;
        float gust = (Mathf.PerlinNoise(t, 0f) - 0.5f) * 2f; // [-1,1]
        float sway = Mathf.Sin(t) * swayAmount * (windMag / 10f) * gust;

        // Rotate opposite wind for natural bend
        Quaternion swayRotation = Quaternion.Euler(
            windDir.z * sway,   // tilt X
            0f,                 // optional Y twist
            -windDir.x * sway   // tilt Z
        );

        targetTransform.localRotation = initialRotation * swayRotation;
    }
}
