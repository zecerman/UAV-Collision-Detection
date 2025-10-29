using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
public class LiDARSensor2 : MonoBehaviour
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
    public LayerMask hitLayers = ~0;
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
        public float x, y, z;   // local to DRONE
        public float dist;      // euclidean from drone origin
        public float az, el;    // angles (relative to THIS sensor's local frame)
        public int hit;         // 1/0
    }

    public int BeamCount => _beams.Count;

    void Awake()
    {
        if (!droneRoot) droneRoot = transform.root;
    }

    void OnEnable()  { BuildBeams(); }
    void OnDisable()
    {
        foreach (var b in _beams) if (b.lr) Destroy(b.lr.gameObject);
        _beams.Clear();
    }

    public void RebuildBeams() => BuildBeams();

    void BuildBeams()
    {
        foreach (var b in _beams) if (b.lr) Destroy(b.lr.gameObject);
        _beams.Clear();

        int azStep = Mathf.Max(1, azimuthStepDeg);
        int elStep = Mathf.Max(1, elevationStepDeg);

        // Build a half-sphere relative to the sensor's local forward
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

        Debug.Log($"{name}: built {_beams.Count} beams (hemisphere={hemisphere})");
    }

    void AddBeam(float az, float el) => _beams.Add(new BeamDef { az = az, el = el, lr = null });

    public List<BeamResult> ScanOnce(float maxRange, float minRange,
                                     LayerMask layers, QueryTriggerInteraction trigger)
    {
        var results = new List<BeamResult>(_beams.Count);
        if (!droneRoot) droneRoot = transform.root;

        Vector3 dronePos = droneRoot.position;
        Quaternion droneRot = droneRoot.rotation; // used for local conversion only

        for (int i = 0; i < _beams.Count; i++)
        {
            var b = _beams[i];

            // IMPORTANT: use the SENSOR's rotation so tilt/aim is respected
            Quaternion q = Quaternion.Euler(b.el, b.az, 0f);
            Vector3 dirWorld = (transform.rotation) * (q * Vector3.forward);
            dirWorld.Normalize();

            Vector3 start = transform.position + dirWorld * selfClearance;

            RaycastHit[] buf = new RaycastHit[8];
            int n = Physics.RaycastNonAlloc(start, dirWorld, buf, maxRange, layers, trigger);
            bool didHit = false;
            float bestDist = float.MaxValue;
            Vector3 end = start + dirWorld * maxRange;

            for (int k = 0; k < n; k++)
            {
                var h = buf[k];
                if (h.collider == null) continue;
                if (h.collider.transform.root == droneRoot) continue;

                if (h.distance < bestDist)
                {
                    bestDist = h.distance;
                    end = h.point;
                    didHit = true;
                }
            }

            // Convert hit point to DRONE local (so CSV is drone-centric)
            Vector3 hitLocal = droneRoot.InverseTransformPoint(end);
            float dist = Mathf.Max(minRange, hitLocal.magnitude);

            results.Add(new BeamResult
            {
                x = hitLocal.x, y = hitLocal.y, z = hitLocal.z,
                dist = dist,
                az = b.az, el = b.el,
                hit = didHit ? 1 : 0
            });

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

