using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

[DisallowMultipleComponent]
public class LiDARLogger : MonoBehaviour
{
    [Header("Sensors")]
    public LiDARSensor topSensor;    // assign "LiDAR Sensor Top"
    public LiDARSensor bottomSensor; // assign "LiDAR Sensor Bottom"

    [Header("Timing")]
    public float scanInterval = 0.5f;

    [Header("Output")]
    public string outputFileBase = "LiDAR_Scan";
    private string _sessionPath;
    private StreamWriter _writer;
    private bool _headerWritten = false;

    [Header("Motors")]
    [Range(1, 6)] public int motorCount = 4;
    public float[] motorStrength = new float[6];

    [Header("Shared Ray Params")]
    public float maxRange = 30f;
    public float minRange = 0.05f;

    [Header("Shared Raycast Settings (applied to all sensors)")]
    public LayerMask environmentLayers = ~0;  // include ground/obstacles; exclude Drone
    public QueryTriggerInteraction triggerMode = QueryTriggerInteraction.Ignore;

    float _nextScanTime;
    float _prevTimestamp = -1f;
    Vector3 _prevPos, _prevVel;
    bool _first = true;
    private const int _perBeamCols = 9;   // x,y,z,dist,azim,elev,hit,hemi,emitterName

    void Awake()
    {
        if (motorStrength == null || motorStrength.Length < 6)
            motorStrength = new float[6];
    }

    void Start()
    {
        // Auto-find sensors by name if not assigned
        if (!topSensor)
        {
            var t = GameObject.Find("LiDAR Sensor Top");
            if (t) topSensor = t.GetComponent<LiDARSensor>();
        }
        if (!bottomSensor)
        {
            var b = GameObject.Find("LiDAR Sensor Bottom");
            if (b) bottomSensor = b.GetComponent<LiDARSensor>();
        }

        // Ensure droneRoot on sensors
        if (topSensor && !topSensor.droneRoot) topSensor.droneRoot = transform;
        if (bottomSensor && !bottomSensor.droneRoot) bottomSensor.droneRoot = transform;

        // Open file now, but write header later when beam counts are known
        var dir = Path.Combine(Application.dataPath, "LiDAR_Logs"); // you switched to Assets
        Directory.CreateDirectory(dir);
        _sessionPath = Path.Combine(dir, $"{outputFileBase}_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");
        var fs = new FileStream(_sessionPath, FileMode.Create, FileAccess.Write, FileShare.Read);
        _writer = new StreamWriter(fs);

        _prevPos = transform.position;
        _prevVel = Vector3.zero;
        _nextScanTime = Time.time;
    }

    void Update()
    {
        if (Time.time >= _nextScanTime)
        {
            DoScanAndWrite();
            _nextScanTime = Time.time + scanInterval;
        }
    }

    void OnApplicationQuit() { CloseWriter(); }
    void OnDestroy()         { CloseWriter();
#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }

    void CloseWriter()
    {
        if (_writer != null) { _writer.Flush(); _writer.Dispose(); _writer = null; }
    }

    void WriteHeader()
    {
        var H = new List<string>
        {
            "Timestamp(s)",
            "yaw(deg)","pitch(deg)","roll(deg)",
            "vx(m_s)","vy(m_s)","vz(m_s)",
            "ax(m_s2)","ay(m_s2)","az(m_s2)"
        };
        for (int m = 1; m <= motorCount; m++) H.Add($"motor{m}.strength");

        int totalBeams =
            (topSensor    ? topSensor.BeamCount    : 0) +
            (bottomSensor ? bottomSensor.BeamCount : 0);

        for (int i = 0; i < totalBeams; i++)
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

        _writer.WriteLine(string.Join(",", H));
        _writer.Flush();
        _headerWritten = true;

        Debug.Log($"LiDAR header written. top={ (topSensor?topSensor.BeamCount:0) } bottom={ (bottomSensor?bottomSensor.BeamCount:0) }");
    }

    void DoScanAndWrite()
    {
        var inv = CultureInfo.InvariantCulture;

        // Make sure sensors exist
        if (!topSensor && !bottomSensor)
        {
            Debug.LogWarning("LiDARLogger: No sensors found. Assign Top/Bottom sensors.");
            return;
        }

        // Make sure beams are built; if not, rebuild them
        if (topSensor && topSensor.BeamCount == 0)    topSensor.RebuildBeams();
        if (bottomSensor && bottomSensor.BeamCount == 0) bottomSensor.RebuildBeams();

        // Only write header once we know how many beams there are
        if (!_headerWritten)
        {
            int total = (topSensor?topSensor.BeamCount:0) + (bottomSensor?bottomSensor.BeamCount:0);
            if (total == 0)
            {
                Debug.LogWarning("LiDARLogger: BeamCount still 0; will try again next frame.");
                return;
            }
            WriteHeader();
        }

        float t = Time.time;
        Vector3 dronePos = transform.position;

        // Unity eulers: x=pitch, y=yaw, z=roll
        Vector3 e = transform.rotation.eulerAngles;
        float yaw = e.y, pitch = e.x, roll = e.z;

        // derivatives
        Vector3 vel = Vector3.zero, acc = Vector3.zero;
        if (!_first)
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

        // SCAN: top then bottom, so ordering is stable
        if (topSensor)
        {
            var rTop = topSensor.ScanOnce(maxRange, minRange, environmentLayers, triggerMode);
            AppendBeams(row, rTop, inv);
        }
        if (bottomSensor)
        {
            var rBot = bottomSensor.ScanOnce(maxRange, minRange, environmentLayers, triggerMode);
            AppendBeams(row, rBot, inv);
        }

        // sanity
        int baseCols = 10 + motorCount; // timestamp + 9 imu/orient + motors
        int perBeam  = 9;
        int expected = baseCols +
                       ((topSensor?topSensor.BeamCount:0) + (bottomSensor?bottomSensor.BeamCount:0)) * perBeam;
        if (row.Count != expected)
            Debug.LogError($"CSV column mismatch: have {row.Count}, expected {expected}");

        _writer.WriteLine(string.Join(",", row));
        _writer.Flush();

        _first = false;
        _prevTimestamp = t;
        _prevPos = dronePos;
        _prevVel = vel;
    }

    static void AppendBeams(List<string> row, List<LiDARSensor.BeamResult> results, CultureInfo inv)
    {
        for (int i = 0; i < results.Count; i++)
        {
            var r = results[i];
            row.Add(r.x.ToString("F4", inv));
            row.Add(r.y.ToString("F4", inv));
            row.Add(r.z.ToString("F4", inv));
            row.Add(r.dist.ToString("F4", inv));
            row.Add(r.az.ToString("F1", inv));
            row.Add(r.el.ToString("F1", inv));
            row.Add(r.hit != 0 ? "1" : "0");
            row.Add(r.hemi);
            row.Add(r.emitterName ?? ""); 
        }
    }
}

