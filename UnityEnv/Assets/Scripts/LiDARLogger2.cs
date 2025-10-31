using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

[DisallowMultipleComponent]
public class LiDARLogger2 : MonoBehaviour
{
    [Header("Sensors")]
    public LiDARSensor2 frontSensor;  // attach the "Front Sensor" object's LiDARSensor2
    public LiDARSensor2 backSensor;   // attach the "Back Sensor" object's LiDARSensor2

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
    public LayerMask environmentLayers = ~0;
    public QueryTriggerInteraction triggerMode = QueryTriggerInteraction.Ignore;

    [Header("Output Location")]
    [Tooltip("Folder name for NEW logs. Created if missing.")]
    public string logsFolderName = "LiDAR_Logs";
    [Tooltip("True: use Application.persistentDataPath (recommended). False: use Assets.")]
    public bool usePersistentPath = true;

    string GetLogsRoot()
    {
        var root = usePersistentPath ? Application.persistentDataPath : Application.dataPath;
        var full = Path.Combine(root, logsFolderName);
        Directory.CreateDirectory(full); // ensures folder exists
        return full;
    }

    string StartNewSessionFile()
    {
        var dir = GetLogsRoot();
        _sessionPath = Path.Combine(dir, $"{outputFileBase}_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");
        var fs = new FileStream(_sessionPath, FileMode.Create, FileAccess.Write, FileShare.Read);
        _writer = new StreamWriter(fs);
        return _sessionPath;
    }

    float _nextScanTime;
    float _prevTimestamp = -1f;
    Vector3 _prevPos, _prevVel;
    bool _first = true;

    private const int _perBeamCols = 7;   // x,y,z,dist,azim,elev,hit
    private bool _headerWritten = false;
    private int  _headerBeamTotal = -1;

    void Awake()
    {
        if (motorStrength == null || motorStrength.Length < 6)
            motorStrength = new float[6];
    }

    void Start()
    {
        // Auto-find by exact names if not assigned in the Inspector.
        if (!frontSensor)
        {
            var f = GameObject.Find("Front Sensor");
            if (f) frontSensor = f.GetComponent<LiDARSensor2>();
        }
        if (!backSensor)
        {
            var b = GameObject.Find("Back Sensor");
            if (b) backSensor = b.GetComponent<LiDARSensor2>();
        }

        // Ensure both sensors know the drone root (this object)
        if (frontSensor && !frontSensor.droneRoot) frontSensor.droneRoot = transform;
        if (backSensor  && !backSensor.droneRoot)  backSensor.droneRoot  = transform;

        // Apply shared physics parameters to both sensors (keeps behavior in sync)
        if (frontSensor)
        {
            frontSensor.hitLayers = environmentLayers;
            frontSensor.triggerInteraction = triggerMode;
            frontSensor.maxRange = maxRange;
            frontSensor.minRange = minRange;
        }
        if (backSensor)
        {
            backSensor.hitLayers = environmentLayers;
            backSensor.triggerInteraction = triggerMode;
            backSensor.maxRange = maxRange;
            backSensor.minRange = minRange;
        }

        StartNewSessionFile();

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
    void OnDestroy()
    {
        CloseWriter();
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

        // 1) SCAN FIRST so we know beam count
        List<LiDARSensor2.BeamResult> rFront = null, rBack = null;
        if (frontSensor)
            rFront = frontSensor.ScanOnce(frontSensor.maxRange, frontSensor.minRange, frontSensor.hitLayers, frontSensor.triggerInteraction);
        if (backSensor)
            rBack  = backSensor.ScanOnce (backSensor.maxRange,  backSensor.minRange,  backSensor.hitLayers,  backSensor.triggerInteraction);

        int beamsNow = (rFront?.Count ?? 0) + (rBack?.Count ?? 0);

        // 2) Ensure header matches
        if (!_headerWritten || _headerBeamTotal != beamsNow)
        {
            if (_headerWritten)
            {
                CloseWriter();
                StartNewSessionFile();
            }
            WriteHeader(beamsNow);
        }

        // 3) Row prefix (state)
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

        // 4) Append beams (front then back)
        if (rFront != null) AppendBeams(row, rFront, inv);
        if (rBack  != null) AppendBeams(row, rBack,  inv);

        // 5) Check count
        int baseCols = 10 + motorCount;
        int expected = baseCols + beamsNow * _perBeamCols;
        if (row.Count != expected)
            Debug.LogError($"CSV column mismatch: have {row.Count}, expected {expected} (beamsNow={beamsNow})");

        // 6) Write
        _writer.WriteLine(string.Join(",", row));
        _writer.Flush();

        _first = false;
        _prevTimestamp = t;
        _prevPos = dronePos;
        _prevVel = vel;
    }

    static void AppendBeams(List<string> row, List<LiDARSensor2.BeamResult> results, CultureInfo inv)
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

