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
    private const int _perBeamCols = 7;   // x,y,z,dist,azim,elev,hit
    private bool _headerWritten = false;
    private int  _headerBeamTotal = -1;   // how many beams the current header was made for

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

    void WriteHeader(int totalBeams)
    {
        var H = new List<string>
        {
            "Timestamp(s)",
            "yaw(deg)","pitch(deg)","roll(deg)",
            "vx(m_s)","vy(m_s)","vz(m_s)",
            "ax(m_s2)","ay(m_s2)","az(m_s2)"
        };
        for (int m = 1; m <= motorCount; m++) H.Add($"motor{m}.strength");

        // 7 columns per beam (no hemi, no emitterName)
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
        }

        _writer.WriteLine(string.Join(",", H));
        _writer.Flush();

        _headerWritten   = true;
        _headerBeamTotal = totalBeams;

        Debug.Log($"LiDAR header written for totalBeams={totalBeams}");
    }

    void DoScanAndWrite()
{
        var inv = CultureInfo.InvariantCulture;

        // 1) SCAN FIRST so we know exactly how many beams we will write this frame
        List<LiDARSensor.BeamResult> rTop = null, rBot = null;
        if (topSensor)
            rTop = topSensor.ScanOnce(maxRange, minRange, topSensor.hitLayers, topSensor.triggerInteraction);
        if (bottomSensor)
            rBot = bottomSensor.ScanOnce(maxRange, minRange, bottomSensor.hitLayers, bottomSensor.triggerInteraction);

        int beamsNow = (rTop?.Count ?? 0) + (rBot?.Count ?? 0);

        // 2) If header not written yet or beam count changed, start a NEW file with matching header
        if (!_headerWritten || _headerBeamTotal != beamsNow)
        {
            if (_headerWritten)
            {
                CloseWriter();
                var dir = Path.Combine(Application.dataPath, "LiDAR_Logs");
                Directory.CreateDirectory(dir);
                _sessionPath = Path.Combine(dir, $"{outputFileBase}_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");
                var fsNew = new FileStream(_sessionPath, FileMode.Create, FileAccess.Write, FileShare.Read);
                _writer = new StreamWriter(fsNew);
            }
            WriteHeader(beamsNow);
        }

        // 3) Build row prefix (timestamp/orientation/derivatives/motors)
        float t = Time.time;
        Vector3 dronePos = transform.position;

        Vector3 e = transform.rotation.eulerAngles; // x=pitch, y=yaw, z=roll
        float yaw = e.y, pitch = e.x, roll = e.z;

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

        // 4) Append beams we actually scanned
        if (rTop != null) AppendBeams(row, rTop, inv);
        if (rBot != null) AppendBeams(row, rBot, inv);

        // 5) Sanity check against THIS row's beam count with 7 cols/beam
        int baseCols = 10 + motorCount;
        int expected = baseCols + beamsNow * _perBeamCols; // _perBeamCols = 7
        if (row.Count != expected)
            Debug.LogError($"CSV column mismatch: have {row.Count}, expected {expected} (beamsNow={beamsNow})");

        // 6) Write row
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
    }
    }
}

