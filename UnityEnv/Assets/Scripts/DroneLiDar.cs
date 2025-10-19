using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

[DisallowMultipleComponent]
public class DroneLiDar : MonoBehaviour
{
    [Header("LiDAR coverage")]
    [Range(5, 60)] public int azimuthStepDeg = 15;   // around Y (0..360)
    [Range(5, 60)] public int elevationStepDeg = 15; // 0..90 per hemisphere

    [Tooltip("Max ray distance (meters).")]
    public float maxRange = 30f;

    [Tooltip("Clamp tiny ranges to avoid zeros near-contact (m).")]
    public float minRange = 0.05f;

    [Tooltip("Seconds between scans.")]
    public float scanInterval = 0.5f;

    [Tooltip("Meters to start ahead of sensor to avoid self-hits.")]
    [Range(0f, 1f)] public float selfClearance = 0.25f;

    [Header("Physics / Raycast")]
    [Tooltip("Include environment layers; exclude your Drone layer.")]
    public LayerMask hitLayers = ~0;
    public QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Ignore;

    [Header("Emitters (auto-find if empty)")]
    public Transform topEmitter;    // child named "LiDAR Sensor Top"
    public Transform bottomEmitter; // child named "LiDAR Sensor Bottom"
    const string TopName = "LiDAR Sensor Top";
    const string BottomName = "LiDAR Sensor Bottom";

    [Header("Visualization")]
    public bool drawBeams = true;
    public Material lineMaterial;
    public float lineWidth = 0.05f;
    public Color missColor = new Color(0.2f, 0.8f, 0.2f);
    public Color hitColor  = new Color(0.0f, 1f, 0.0f);

    [Header("Output")]
    [Tooltip("Base name; file is timestamped under persistentDataPath/LiDAR_Logs.")]
    public string outputFileBase = "LiDAR_Scan";

    [Header("Motors")]
    [Range(1, 6)] public int motorCount = 4;
    public float[] motorStrength = new float[6]; // first motorCount used

    struct Beam
    {
        public float azimuthDeg;
        public float elevationDeg;
        public Transform emitter; // which sensor fires this beam
        public LineRenderer lr;
    }
    readonly List<Beam> beams = new();

    string _sessionPath;
    StreamWriter _writer;
    float _nextScanTime;
    float _prevTimestamp = -1f;
    Vector3 _prevPos, _prevVel;
    bool _firstScan = true;

    void Awake()
    {
        if (motorStrength == null || motorStrength.Length < 6)
            motorStrength = new float[6];
    }

    void Start()
    {
        AutoFindEmitters();
        EnsureFallbackEmitters();
        BuildBeams();
        OpenLogAndWriteHeader();

        _prevPos = transform.position;
        _prevVel = Vector3.zero;
        _nextScanTime = Time.time;
    }

    void Update()
    {
        if (Time.time >= _nextScanTime)
        {
            DoScan();
            _nextScanTime = Time.time + scanInterval;
        }
    }

    void OnApplicationQuit() { CloseWriter(); }
    void OnDestroy()         { CloseWriter(); }

    void AutoFindEmitters()
    {
        if (!topEmitter)    { var t = transform.Find(TopName);    if (t) topEmitter = t; }
        if (!bottomEmitter) { var b = transform.Find(BottomName); if (b) bottomEmitter = b; }
    }

    void EnsureFallbackEmitters()
    {
        if (!topEmitter)
        {
            var go = new GameObject(TopName);
            go.transform.SetParent(transform, false);
            go.transform.localPosition = new Vector3(0f, 0.25f, 0f);
            topEmitter = go.transform;
        }
        if (!bottomEmitter)
        {
            var go = new GameObject(BottomName);
            go.transform.SetParent(transform, false);
            go.transform.localPosition = new Vector3(0f, -0.25f, 0f);
            bottomEmitter = go.transform;
        }
    }

    void BuildBeams()
    {
        foreach (var b in beams) if (b.lr) Destroy(b.lr.gameObject);
        beams.Clear();

        int azStep = Mathf.Max(1, azimuthStepDeg);
        int elStep = Mathf.Max(1, elevationStepDeg);

        // Upper hemisphere (+elev) from TOP emitter
        for (int az = 0; az < 360; az += azStep)
            for (int el = elStep; el <= 90; el += elStep)
                beams.Add(new Beam { azimuthDeg = az, elevationDeg = +el, emitter = topEmitter });

        // Lower hemisphere (-elev) from BOTTOM emitter
        for (int az = 0; az < 360; az += azStep)
            for (int el = -elStep; el >= -90; el -= elStep)
                beams.Add(new Beam { azimuthDeg = az, elevationDeg = el, emitter = bottomEmitter });

        if (drawBeams)
        {
            for (int i = 0; i < beams.Count; i++)
            {
                var go = new GameObject($"LiDAR_Beam_{i:D3}");
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

                var b = beams[i]; b.lr = lr; beams[i] = b;
            }
        }

        // Sanity
        int topCount = 0, botCount = 0;
        foreach (var b in beams) if (b.emitter == bottomEmitter) botCount++; else topCount++;
        Debug.Log($"LiDAR beams — top:{topCount} bottom:{botCount} total:{beams.Count}");
    }

    void OpenLogAndWriteHeader()
    {
        var dir = Path.Combine(Application.persistentDataPath, "LiDAR_Logs");
        Directory.CreateDirectory(dir);
        _sessionPath = Path.Combine(dir, $"{outputFileBase}_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");

        var fs = new FileStream(_sessionPath, FileMode.Create, FileAccess.Write, FileShare.Read);
        _writer = new StreamWriter(fs);
        WriteHeader(_writer);
        _writer.Flush();
        Debug.Log($"LiDAR logging to: {_sessionPath}");
    }

    void CloseWriter()
    {
        if (_writer != null) { _writer.Flush(); _writer.Dispose(); _writer = null; }
    }

    void WriteHeader(StreamWriter sw)
    {
        var H = new List<string>
        {
            "Timestamp(s)",
            "yaw(deg)","pitch(deg)","roll(deg)",
            "vx(m_s)","vy(m_s)","vz(m_s)",
            "ax(m_s2)","ay(m_s2)","az(m_s2)"
        };
        for (int m = 1; m <= motorCount; m++) H.Add($"motor{m}.strength");

        // 9 columns per beam: x,y,z,dist,azim,elev,hit,hemi,emitterName
        for (int i = 0; i < beams.Count; i++)
        {
            int k = i + 1;
            H.Add($"beam{k}.x(m)");
            H.Add($"beam{k}.y(m)");
            H.Add($"beam{k}.z(m)");
            H.Add($"beam{k}.dist(m)");
            H.Add($"beam{k}.azim(deg)");
            H.Add($"beam{k}.elev(deg)");
            H.Add($"beam{k}.hit(0/1)");
            H.Add($"beam{k}.hemi");
            H.Add($"beam{k}.emitterName");
        }
        sw.WriteLine(string.Join(",", H));
    }

    void DoScan()
    {
        var inv = CultureInfo.InvariantCulture;

        float t = Time.time;
        Vector3 dronePos = transform.position;

        // Unity Euler: x=pitch, y=yaw, z=roll
        Vector3 e = transform.rotation.eulerAngles;
        float yaw = e.y, pitch = e.x, roll = e.z;

        Vector3 vel = Vector3.zero, acc = Vector3.zero;
        if (!_firstScan)
        {
            float dt = Mathf.Max(1e-6f, t - _prevTimestamp);
            vel = (dronePos - _prevPos) / dt;
            acc = (vel - _prevVel) / dt;
        }

        var row = new List<string>
        {
            t.ToString("F3", inv),
            yaw.ToString("F3", inv), pitch.ToString("F3", inv), roll.ToString("F3", inv),
            vel.x.ToString("F4", inv), vel.y.ToString("F4", inv), vel.z.ToString("F4", inv),
            acc.x.ToString("F4", inv), acc.y.ToString("F4", inv), acc.z.ToString("F4", inv)
        };

        for (int m = 0; m < motorCount; m++)
            row.Add((m < motorStrength.Length ? motorStrength[m] : 0f).ToString("F3", inv));

        // ---- Per-beam ----
        for (int i = 0; i < beams.Count; i++)
        {
            var b = beams[i];

            // Use DRONE orientation for direction (consistent for both sensors)
            Quaternion qDrone = Quaternion.Euler(b.elevationDeg, b.azimuthDeg, 0f);
            Vector3 dirWorld  = transform.rotation * (qDrone * Vector3.forward);
            dirWorld.Normalize();

            // Start at the sensor's position with a small clearance
            Vector3 start = b.emitter.position + dirWorld * selfClearance;

            // Single raycast; self-hits prevented by LayerMask (Drone unchecked)
            bool didHit = Physics.Raycast(start, dirWorld, out RaycastHit hit, maxRange, hitLayers, triggerInteraction);

            Vector3 end  = didHit ? hit.point : (start + dirWorld * maxRange);
            float   dist = didHit ? Mathf.Max(minRange, hit.distance) : maxRange;

            // Local coords (relative to DRONE origin, rotation-only)
            Vector3 hitLocal = Vector3.zero;
            if (didHit)
            {
                Vector3 toHitFromDrone = end - dronePos;
                hitLocal = Quaternion.Inverse(transform.rotation) * toHitFromDrone;
            }

            // Append 9 cells per beam (ALWAYS)
            row.Add(hitLocal.x.ToString("F4", inv));
            row.Add(hitLocal.y.ToString("F4", inv));
            row.Add(hitLocal.z.ToString("F4", inv));
            row.Add(dist.ToString("F4", inv));
            row.Add(b.azimuthDeg.ToString("F1", inv));
            row.Add(b.elevationDeg.ToString("F1", inv));
            row.Add(didHit ? "1" : "0");
            row.Add(b.emitter == bottomEmitter ? "bottom" : "top");
            row.Add(b.emitter ? b.emitter.name : "");

            // Visual
            if (drawBeams && b.lr)
            {
                b.lr.enabled = true;
                b.lr.useWorldSpace = true;
                b.lr.SetPosition(0, start);
                b.lr.SetPosition(1, end);
                var c = didHit ? hitColor : missColor;
                b.lr.startColor = c; b.lr.endColor = c;
            }
        }

        // Column-count sanity check
        int baseCols = 10 + motorCount; // timestamp + 9 imu/orient + motors
        int perBeam  = 9;               // x,y,z,dist,azim,elev,hit,hemi,emitterName
        int expected = baseCols + beams.Count * perBeam;
        if (row.Count != expected)
            Debug.LogError($"CSV column mismatch: have {row.Count}, expected {expected}");

        _writer.WriteLine(string.Join(",", row));
        _writer.Flush();

        _firstScan = false;
        _prevTimestamp = t;
        _prevPos = dronePos;
        _prevVel = vel;
    }
}



