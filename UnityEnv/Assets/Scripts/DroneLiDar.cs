using System.Collections.Generic;
using System.IO;
using UnityEngine;

[DisallowMultipleComponent]
public class DroneLiDAR : MonoBehaviour
{
    [Header("LiDAR coverage (hemispheres)")]
    [Range(5, 60)] public int azimuthStepDeg = 15;     // 15–30 typical
    [Range(5, 60)] public int elevationStepDeg = 15;   // 15–30 typical
    public float maxRange = 30f;
    public float minRange = 0.05f;
    public float scanInterval = 0.5f;

    [Header("Physics / Raycast")]
    public LayerMask hitLayers = ~0;
    public QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Ignore;

    [Header("Visualization")]
    public bool drawBeams = true;
    public Material lineMaterial;
    public float lineWidth = 0.02f;

    [Header("Output")]
    public string outputFileName = "LiDAR_Scan.csv";
    [Tooltip("Also log azimuth/elevation and hit flag per beam.")]
    public bool logBeamAnglesAndHit = true;

    [Header("Motors")]
    [Range(1, 6)] public int motorCount = 4;
    public float[] motorStrength = new float[6];

    // internals
    private string filePath;
    private float nextScanTime;
    private float prevTimestamp = -1f;
    private Vector3 prevPos, prevVel;
    private bool firstScan = true;

    private struct Beam
    {
        public float azimuthDeg;   // around +Y
        public float elevationDeg; // +up (upper hemi), −down (lower)
        public Vector3 localDir;   // unit vector in drone local space
        public LineRenderer lr;    // optional visual
    }
    private readonly List<Beam> beams = new();

    void Awake()
    {
        if (motorStrength == null || motorStrength.Length < 6)
            motorStrength = new float[6];
    }

    void Start()
    {
        filePath = Path.Combine(Application.dataPath, outputFileName);
        BuildBeams();
        WriteHeader();
        prevPos = transform.position;
        prevVel = Vector3.zero;
        nextScanTime = Time.time;
    }

    void Update()
    {
        if (Time.time >= nextScanTime)
        {
            DoScan();
            nextScanTime = Time.time + scanInterval;
        }
    }

    void BuildBeams()
    {
        // clear old visuals
        foreach (var b in beams) if (b.lr != null) Destroy(b.lr.gameObject);
        beams.Clear();

        int azStep = Mathf.Max(1, azimuthStepDeg);
        int elStep = Mathf.Max(1, elevationStepDeg);

        for (int az = 0; az < 360; az += azStep)
        {
            // upper hemisphere (skip exactly 0 to avoid horizon dup)
            for (int el = elStep; el <= 90; el += elStep) AddBeam(az, +el);
            // lower hemisphere
            for (int el = -elStep; el >= -90; el -= elStep) AddBeam(az, el);
        }

        if (drawBeams)
        {
            for (int i = 0; i < beams.Count; i++)
            {
                var go = new GameObject($"LiDAR_Beam_{i:D3}");
                go.transform.SetParent(transform, false);
                var lr = go.AddComponent<LineRenderer>();
                lr.material = lineMaterial != null ? lineMaterial : new Material(Shader.Find("Sprites/Default"));
                lr.positionCount = 2;
                lr.startWidth = lineWidth;
                lr.endWidth = lineWidth;
                var b = beams[i]; b.lr = lr; beams[i] = b;
            }
        }
    }

    void AddBeam(float azimuthDeg, float elevationDeg)
    {
        Quaternion q = Quaternion.Euler(elevationDeg, azimuthDeg, 0f);
        Vector3 localDir = (q * Vector3.forward).normalized;
        beams.Add(new Beam { azimuthDeg = azimuthDeg, elevationDeg = elevationDeg, localDir = localDir, lr = null });
    }

    void WriteHeader()
    {
        using var sw = new StreamWriter(filePath, false);
        var headers = new List<string>
        {
            "Timestamp(s)",
            "yaw(deg)","pitch(deg)","roll(deg)",
            "vx(m_s)","vy(m_s)","vz(m_s)",
            "ax(m_s2)","ay(m_s2)","az(m_s2)"
        };

        for (int m = 1; m <= motorCount; m++)
            headers.Add($"motor{m}.strength");

        for (int i = 0; i < beams.Count; i++)
        {
            int k = i + 1;
            headers.Add($"beam{k}.dist(m)");  // non-negative range only
            if (logBeamAnglesAndHit)
            {
                headers.Add($"beam{k}.azim(deg)");
                headers.Add($"beam{k}.elev(deg)");
                headers.Add($"beam{k}.hit(0/1)");
            }
        }
        sw.WriteLine(string.Join(",", headers));
    }

    void DoScan()
    {
        float t = Time.time;
        Vector3 pWorld = transform.position;

        // orientation (Unity: x=pitch, y=yaw, z=roll)
        Vector3 e = transform.rotation.eulerAngles;
        float yaw = e.y, pitch = e.x, roll = e.z;

        // derivatives with real Δt
        Vector3 vel = Vector3.zero, acc = Vector3.zero;
        if (!firstScan)
        {
            float dt = Mathf.Max(1e-6f, t - prevTimestamp);
            vel = (pWorld - prevPos) / dt;
            acc = (vel - prevVel) / dt;
        }

        var row = new List<string>
        {
            t.ToString("F3"),
            yaw.ToString("F3"), pitch.ToString("F3"), roll.ToString("F3"),
            vel.x.ToString("F4"), vel.y.ToString("F4"), vel.z.ToString("F4"),
            acc.x.ToString("F4"), acc.y.ToString("F4"), acc.z.ToString("F4")
        };

        for (int m = 0; m < motorCount; m++)
            row.Add((m < motorStrength.Length ? motorStrength[m] : 0f).ToString("F3"));

        // raycast each beam; log range only
        for (int i = 0; i < beams.Count; i++)
        {
            var b = beams[i];
            Vector3 dirWorld = transform.TransformDirection(b.localDir);

            bool didHit = Physics.Raycast(
                pWorld, dirWorld, out RaycastHit hit,
                maxRange, hitLayers, triggerInteraction);

            // non-negative distance along the ray:
            float dist = didHit ? hit.distance : maxRange;   // Unity returns ≥0
            if (dist < minRange) dist = minRange;

            row.Add(dist.ToString("F4"));

            if (logBeamAnglesAndHit)
            {
                row.Add(b.azimuthDeg.ToString("F1"));
                row.Add(b.elevationDeg.ToString("F1"));
                row.Add(didHit ? "1" : "0");
            }

            if (drawBeams && b.lr != null)
            {
                Vector3 end = didHit ? hit.point : (pWorld + dirWorld * maxRange);
                b.lr.SetPosition(0, pWorld);
                b.lr.SetPosition(1, end);
                var c = didHit ? Color.green : new Color(0.2f, 0.8f, 0.2f);
                b.lr.startColor = c; b.lr.endColor = c;
            }
        }

        using (var sw = new StreamWriter(filePath, true))
            sw.WriteLine(string.Join(",", row));

        firstScan = false;
        prevTimestamp = t;
        prevPos = pWorld;
        prevVel = vel;
    }
}
