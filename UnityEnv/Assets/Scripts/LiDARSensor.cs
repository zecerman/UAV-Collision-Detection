using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
public class LiDARSensor : MonoBehaviour
{
    public enum Hemisphere { Top, Bottom }

    [Header("Sensor Setup")]
    public Hemisphere hemisphere = Hemisphere.Top;

    [Tooltip("Reference to the DRONE root (the one that holds the logger).")]
    public Transform droneRoot;

    [Header("Coverage")]
    [Range(5, 60)] public int azimuthStepDeg = 15;
    [Range(5, 60)] public int elevationStepDeg = 15;

    [Tooltip("Meters to start ahead of sensor to avoid self-hits.")]
    [Range(0f, 1f)] public float selfClearance = 0.25f;

    [Header("Physics / Raycast")]
    public LayerMask hitLayers = ~0;  // exclude Drone in the mask
    public QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Ignore;

    [Header("Visualization")]
    public bool drawBeams = true;
    public Material lineMaterial;
    public float lineWidth = 0.05f;
    public Color missColor = new Color(0.2f, 0.8f, 0.2f);
    public Color hitColor  = new Color(0.0f, 1f, 0.0f);

    [Header("Ranges")]
    public float maxRange = 30f;
    public float minRange = 0.05f;

    struct BeamDef { public float az, el; public LineRenderer lr; }
    List<BeamDef> _beams = new List<BeamDef>();

    public struct BeamResult
    {
        public float x, y, z;     // local to DRONE
        public float dist;
        public float az, el;
        public int hit;           // 1/0
        public string hemi;       // "top"/"bottom"
        public string emitterName;
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
    public void RebuildBeams()
    {
        BuildBeams();
    }

    void BuildBeams()
    {
        foreach (var b in _beams) if (b.lr) Destroy(b.lr.gameObject);
        _beams.Clear();

        int azStep = Mathf.Max(1, azimuthStepDeg);
        int elStep = Mathf.Max(1, elevationStepDeg);

        if (hemisphere == Hemisphere.Top)
        {
            for (int az = 0; az < 360; az += azStep)
                for (int el = elStep; el <= 90; el += elStep)
                    AddBeam(az, +el);
        }
        else
        {
            for (int az = 0; az < 360; az += azStep)
                for (int el = -elStep; el >= -90; el -= elStep)
                    AddBeam(az, el);
        }

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
                    var sh = Shader.Find("Unlit/Color");
                    lr.material = new Material(sh ? sh : Shader.Find("Sprites/Default"));
                    if (sh) lr.material.SetColor("_Color", Color.green);
                }
                lr.positionCount = 2;
                lr.useWorldSpace = true;
                lr.startWidth = lineWidth;
                lr.endWidth   = lineWidth;
                lr.numCapVertices = 4;

                var b = _beams[i]; b.lr = lr; _beams[i] = b;
            }
        }

        Debug.Log($"{name}: built {_beams.Count} beams for {hemisphere} hemisphere");
    }

    void AddBeam(float az, float el) => _beams.Add(new BeamDef { az = az, el = el, lr = null });

    /// Perform a scan for this sensor and return results for each beam.
    /// Uses the DRONE's orientation for direction, and this sensor's position as origin.
    public List<BeamResult> ScanOnce(float maxRange, float minRange,
                                     LayerMask layers, QueryTriggerInteraction trigger)
    {
        var results = new List<BeamResult>(_beams.Count);
        if (!droneRoot) droneRoot = transform.root;

        Vector3 dronePos = droneRoot.position;
        Quaternion droneRot = droneRoot.rotation;

        for (int i = 0; i < _beams.Count; i++)
        {
            var b = _beams[i];

            // Direction in DRONE frame (consistent for top/bottom)
            Quaternion q = Quaternion.Euler(b.el, b.az, 0f);
            Vector3 dirWorld = droneRot * (q * Vector3.forward);
            dirWorld.Normalize();

            Vector3 start = transform.position + dirWorld * selfClearance;

            // --- Robust raycast that skips self-hits ---
            RaycastHit hit;
            bool didHit = false;

            // Use a small buffer so we can ignore our own colliders safely
            RaycastHit[] buf = new RaycastHit[8];
            int n = Physics.RaycastNonAlloc(start, dirWorld, buf, maxRange, layers, trigger);
            float bestDist = float.MaxValue;
            Vector3 end = start + dirWorld * maxRange;

            for (int k = 0; k < n; k++)
            {
                var h = buf[k];
                if (h.collider == null) continue;
                // Skip any collider that belongs to the same root (drone/sensors)
                if (h.collider.transform.root == droneRoot) continue;

                if (h.distance < bestDist)
                {
                    bestDist = h.distance;
                    hit = h;
                    end = h.point;
                    didHit = true;
                }
            }

            // --- Compute local coords RELATIVE TO DRONE ORIGIN (full TRS) ---
            Vector3 hitLocal = droneRoot.InverseTransformPoint(end);

            // Euclidean distance from drone origin (NOT start)
            float dist = hitLocal.magnitude;
            if (didHit) dist = Mathf.Max(minRange, dist); else dist = Mathf.Max(minRange, dist); // keep consistent min clamp

            // Package result
            results.Add(new BeamResult
            {
                x = hitLocal.x,
                y = hitLocal.y,
                z = hitLocal.z,
                dist = dist,
                az = b.az,
                el = b.el,
                hit = didHit ? 1 : 0,
                hemi = hemisphere == Hemisphere.Top ? "top" : "bottom",
                emitterName = name
            });

            // visuals
            if (drawBeams && b.lr)
            {
                b.lr.enabled = true;
                b.lr.SetPosition(0, start);
                b.lr.SetPosition(1, end);
                var c = didHit ? hitColor : missColor;
                b.lr.startColor = c; b.lr.endColor = c;
             }
        }

        return results;
    }
}
