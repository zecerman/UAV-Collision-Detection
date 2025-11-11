using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
public class LiDARSensor : MonoBehaviour
{
    public enum Hemisphere { Top, Bottom }

    [Tooltip("Reference to the DRONE root (the one that holds the logger).")]
    public Transform droneRoot;

    [Header("Coverage")]
    [Range(5, 60)] public int azimuthStepDeg = 30;
    [Range(5, 60)] public int elevationStepDeg = 30;

    [Tooltip("Meters to start ahead of sensor to avoid self-hits.")]
    [Range(0f, 1f)] public float selfClearance = 0.25f;

    [Header("Physics / Raycast")]
    public LayerMask hitLayers = ~0;  // exclude Drone in the mask
    public QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Ignore;

    [Header("Visualization")]
    public bool drawBeams = true;
    public Material lineMaterial;
    public float lineWidth = 0.05f;
    public Color missColor = new Color(1f, 0.0f, 0.0f);
    public Color hitColor  = new Color(0.0f, 1f, 0.0f);

    [Header("Ranges")]
    public float maxRange = 30f;
    public float minRange = 0.05f;

    struct BeamDef { public float az, el; public LineRenderer lr; }
    List<BeamDef> _beams = new List<BeamDef>();

    public struct BeamResult
    {
        public float x, y, z;   // local to DRONE
        public float dist;      // euclidean from drone origin
        public float az, el;    // angles in degrees
        public int hit;         // 1/0
    }

    public int BeamCount => _beams.Count;

    void Awake()
    {
        if (!droneRoot) droneRoot = transform.root;
    }

    void OnEnable() { BuildBeams(); }
    void OnDisable()
    {
        foreach (var b in _beams) if (b.lr) Destroy(b.lr.gameObject);
        _beams.Clear();
    }

    // Expose for logger to force a rebuild if needed
    public void RebuildBeams() => BuildBeams();

    void BuildBeams()
    {
        foreach (var b in _beams) if (b.lr) Destroy(b.lr.gameObject);
        _beams.Clear();

        int azStep = Mathf.Max(1, azimuthStepDeg);
        int elStep = Mathf.Max(1, elevationStepDeg);

        for (int az = 0; az < 360; az += azStep)
            for (int el = 0; el <= 90; el += elStep)
                AddBeam(az, el);

        if (drawBeams)
        {
            for (int i = 0; i < _beams.Count; i++)
            {
                var go = new GameObject($"{name}_Beam_{i:D3}");
                go.transform.SetParent(transform, false);
                var lr = go.AddComponent<LineRenderer>();
                if (lineMaterial != null) lr.material = lineMaterial;
                else
                {
                    var sh = Shader.Find("Sprites/Default")
                             ?? Shader.Find("Universal Render Pipeline/Unlit");
                    lr.material = new Material(sh);
                    lr.material.color = Color.white;
                }
                lr.positionCount = 2;
                lr.useWorldSpace = true;
                lr.startWidth = lineWidth;
                lr.endWidth   = lineWidth;
                lr.numCapVertices = 4;

                var b = _beams[i]; b.lr = lr; _beams[i] = b;
            }
        }

        Debug.Log($"{name}: built {_beams.Count} beams");
    }

    void AddBeam(float az, float el) => _beams.Add(new BeamDef { az = az, el = el, lr = null });

    /// Perform a scan for this sensor and return results for each beam.
    /// Uses the DRONE's orientation for direction, and this sensor's position as origin.
    public List<BeamResult> ScanOnce(float maxRangeOverride, float minRangeOverride,
                                     LayerMask layers, QueryTriggerInteraction trigger)
    {
        float useMax = maxRangeOverride > 0 ? maxRangeOverride : maxRange;
        float useMin = minRangeOverride > 0 ? minRangeOverride : minRange;

        var results = new List<BeamResult>(_beams.Count);
        if (!droneRoot) droneRoot = transform.root;

        for (int i = 0; i < _beams.Count; i++)
        {
            var b = _beams[i];

            // Use the sensor's transform as sweep basis
            Transform basis = transform;

            // Axis of the cone = parent's up
            Vector3 u = basis.up.normalized;

            // Orthonormal frame around u (e1,e2 lie in the plane perpendicular to u)
            Vector3 e1 = Vector3.ProjectOnPlane(basis.forward, u);
            if (e1.sqrMagnitude < 1e-6f) e1 = Vector3.ProjectOnPlane(basis.right, u);
            e1.Normalize();
            Vector3 e2 = Vector3.Cross(u, e1);

            float theta = Mathf.Deg2Rad * Mathf.Clamp(Mathf.Abs(b.el), 0f, 90f);
            float phi   = Mathf.Deg2Rad * b.az;

            Vector3 dirWorld =
                Mathf.Cos(theta) * u +
                Mathf.Sin(theta) * (Mathf.Cos(phi) * e1 + Mathf.Sin(phi) * e2);
            dirWorld.Normalize();

            Vector3 start = transform.position + dirWorld * selfClearance;

            // --- Robust raycast that skips self-hits ---
            RaycastHit[] buf = new RaycastHit[8];
            int n = Physics.RaycastNonAlloc(start, dirWorld, buf, useMax, layers, trigger);
            float bestDist = float.MaxValue;
            Vector3 end = start + dirWorld * useMax;
            bool didHit = false;

            for (int k = 0; k < n; k++)
            {
                var h = buf[k];
                if (h.collider == null) continue;
                if (h.collider.transform.root == droneRoot) continue; // skip self

                if (h.distance < bestDist)
                {
                    bestDist = h.distance;
                    end = h.point;
                    didHit = true;
                }
            }

            // Compute local coords RELATIVE TO DRONE ORIGIN (full TRS)
            Vector3 hitLocal = droneRoot.InverseTransformPoint(end);

            // Euclidean distance from drone origin
            float dist = Mathf.Max(useMin, hitLocal.magnitude);

            results.Add(new BeamResult
            {
                x = hitLocal.x, y = hitLocal.y, z = hitLocal.z,
                dist = dist,
                az = b.az, el = b.el,
                hit = didHit ? 1 : 0
            });

            // visuals
            if (drawBeams && _beams[i].lr)
            {
                var lr = _beams[i].lr;
                lr.enabled = true;
                lr.SetPosition(0, start);
                lr.SetPosition(1, end);
                var c = didHit ? hitColor : missColor;
                lr.startColor = c; lr.endColor = c;
            }
        }

        return results;
    }
}
