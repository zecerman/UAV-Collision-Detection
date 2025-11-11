using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

[DefaultExecutionOrder(-1000)] // ensure this runs first
[DisallowMultipleComponent]
public class LiDARLogger : MonoBehaviour
{
    [Header("Sensors")]
    public LiDARSensor topSensor;    // assign "LiDAR Sensor Top"
    public LiDARSensor bottomSensor; // assign "LiDAR Sensor Bottom"

    [Header("Timing")]
    [Tooltip("Seconds between scans. Set small (e.g., 0.1) for training.")]
    public float scanInterval = 0.5f;

    [Header("Live Access (read by DroneAgent)")]
    // Schema of latestRow (unchanged for your agent): [ t, yaw, pitch, roll, vx, vy, vz, ax, ay, az, motor1..motor6, beam_distances... ]
    public float[] latestRow;            // ALWAYS initialized at Start()
    public float[] latestDistances;      // distances only, length = beamCount (convenience)

    [Header("Motors")]
    [Tooltip("If assigned, strengths are pulled from this controller each scan; else uses motorStrength array.")]
    public MotorController motorController;   // optional; must expose float[] GetMotorStrengths()
    [Range(6,6)] public int motorCount = 6;   // fixed at 6 props
    public float[] motorStrength = new float[6]; // fallback / also read by your Agent if needed

    [Header("Shared Ray Params")]
    public float maxRange = 30f;
    public float minRange = 0.05f;

    [Header("Shared Raycast Settings (applied to all sensors)")]
    public LayerMask environmentLayers = ~0;  // include ground/obstacles; exclude Drone
    public QueryTriggerInteraction triggerMode = QueryTriggerInteraction.Ignore;

    [Header("CSV Logging (lean distances + props)")]
    public bool enableCsvLogging = true;
    [Tooltip("If true, write maxRange when a ray misses; if false, write empty cell.")]
    public bool writeMaxRangeOnMiss = true;
    public string outputFileBase = "LiDAR_Scan";
    private string _sessionPath;
    private StreamWriter _writer;

    // --- internals ---
    float _nextScanTime;
    float _prevTimestamp = -1f;
    Vector3 _prevPos, _prevVel;
    bool _first = true;

    // fixed-column header portion counts (for latestRow only; CSV ignores these extra fields)
    const int HeaderFloatCount = 10;  // t, yaw, pitch, roll, vx, vy, vz, ax, ay, az
    int _beamCount = 0;               // computed at Start from sensors
    int _rowLen = 0;                  // HeaderFloatCount + motorCount + _beamCount

    // cached header angles for CSV columns
    private readonly List<float> _headerElev = new(); // degrees
    private readonly List<float> _headerAzim = new(); // degrees

    void Awake()
    {
        if (motorStrength == null || motorStrength.Length < motorCount)
            motorStrength = new float[motorCount];

        // Auto-find sensors if needed
        if (!topSensor)    { var t = GameObject.Find("LiDAR Sensor Top");    if (t) topSensor = t.GetComponent<LiDARSensor>(); }
        if (!bottomSensor) { var b = GameObject.Find("LiDAR Sensor Bottom"); if (b) bottomSensor = b.GetComponent<LiDARSensor>(); }

        // Ensure roots
        if (topSensor    && !topSensor.droneRoot)    topSensor.droneRoot    = transform;
        if (bottomSensor && !bottomSensor.droneRoot) bottomSensor.droneRoot = transform;

        // Build beams now so counts are valid
        if (topSensor)    topSensor.RebuildBeams();
        if (bottomSensor) bottomSensor.RebuildBeams();

        // Compute & allocate
        _beamCount = (topSensor ? topSensor.BeamCount : 0) + (bottomSensor ? bottomSensor.BeamCount : 0);
        _rowLen = HeaderFloatCount + motorCount + _beamCount;

        latestDistances = new float[_beamCount];
        for (int i = 0; i < _beamCount; i++) latestDistances[i] = maxRange;

        latestRow = new float[_rowLen];
        WriteDistancesIntoLatestRow();

        // Seed one real scan immediately (no CSV yet)
        _prevPos = transform.position;
        _prevVel = Vector3.zero;
        DoScan(updateCsv: false);

        // Start schedule
        _nextScanTime = Time.time + scanInterval;

        Debug.Log($"[LiDARLogger] Awake: beams={_beamCount}, rowLen={_rowLen}");
    }

    void Start()
    {
        // Redundant safety
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
        if (topSensor    && !topSensor.droneRoot)    topSensor.droneRoot    = transform;
        if (bottomSensor && !bottomSensor.droneRoot) bottomSensor.droneRoot = transform;

        if (topSensor)    topSensor.RebuildBeams();
        if (bottomSensor) bottomSensor.RebuildBeams();

        _beamCount = (topSensor ? topSensor.BeamCount : 0) + (bottomSensor ? bottomSensor.BeamCount : 0);
        _rowLen = HeaderFloatCount + motorCount + _beamCount;

        latestRow = new float[_rowLen];
        latestDistances = new float[_beamCount];
        for (int i = 0; i < _beamCount; i++) latestDistances[i] = maxRange;
        WriteDistancesIntoLatestRow();

        // Prepare CSV if enabled — build header labels from a one-time scan to capture angles
        if (enableCsvLogging)
        {
            var dir = Path.Combine(Application.dataPath, "LiDAR_Logs");
            Directory.CreateDirectory(dir);
            _sessionPath = Path.Combine(dir, $"{outputFileBase}_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");
            var fs = new FileStream(_sessionPath, FileMode.Create, FileAccess.Write, FileShare.Read);
            _writer = new StreamWriter(fs);

            BuildHeaderAnglesFromOneScan();
            WriteCsvHeaderWithProps();  // Timestamp + Prop 1..6 + beamN distance (elev=.. | azim=..)
            _writer.Flush();
        }

        _prevPos = transform.position;
        _prevVel = Vector3.zero;

        // First real scan (and row) if logging
        DoScan(updateCsv: enableCsvLogging);

        _nextScanTime = Time.time + scanInterval;
    }

    void Update()
    {
        if (Time.time >= _nextScanTime)
        {
            DoScan(updateCsv: enableCsvLogging);
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

    // ---------- Build header labels (angles) from a single scan ----------
    void BuildHeaderAnglesFromOneScan()
    {
        _headerElev.Clear();
        _headerAzim.Clear();

        if (topSensor)
        {
            var rTop = topSensor.ScanOnce(maxRange, minRange, topSensor.hitLayers, topSensor.triggerInteraction);
            for (int i = 0; i < rTop.Count; i++) { _headerElev.Add(rTop[i].el); _headerAzim.Add(rTop[i].az); }
        }
        if (bottomSensor)
        {
            var rBot = bottomSensor.ScanOnce(maxRange, minRange, bottomSensor.hitLayers, bottomSensor.triggerInteraction);
            for (int i = 0; i < rBot.Count; i++) { _headerElev.Add(rBot[i].el); _headerAzim.Add(rBot[i].az); }
        }

        // Guard against missing sensor
        while (_headerElev.Count < _beamCount) _headerElev.Add(0f);
        while (_headerAzim.Count < _beamCount) _headerAzim.Add(0f);
    }

    // ---------- CSV HEADER (with props) ----------
    void WriteCsvHeaderWithProps()
    {
        // One row only:
        // Timestamp(s), Prop 1..6, beam1 distance (elev=E° | azim=A°), beam2 distance (...), ...
        var H = new List<string>(1 + motorCount + _beamCount) { "Timestamp(s)" };

        for (int m = 1; m <= motorCount; m++)
            H.Add($"Prop {m}");

        for (int i = 0; i < _beamCount; i++)
        {
            int k = i + 1;
            // Avoid commas inside labels so Excel/Sheets don't split columns
            H.Add($"beam{k} distance (elev={_headerElev[i]:F1}° | azim={_headerAzim[i]:F1}°)");
        }
        _writer.WriteLine(string.Join(",", H));
    }

    // ---------- SCAN ----------
    void DoScan(bool updateCsv)
    {
        var inv = CultureInfo.InvariantCulture;

        // 0) Update motor strengths for this tick
        PullMotorStrengths();   // fills motorStrength[0..5]

        // 1) Scan beams (distances only for CSV)
        int idx = 0;
        if (topSensor)
        {
            var rTop = topSensor.ScanOnce(maxRange, minRange, topSensor.hitLayers, topSensor.triggerInteraction);
            EnsureDistancesLen();
            for (int i = 0; i < rTop.Count; i++) latestDistances[idx++] = rTop[i].dist;
        }
        if (bottomSensor)
        {
            var rBot = bottomSensor.ScanOnce(maxRange, minRange, bottomSensor.hitLayers, bottomSensor.triggerInteraction);
            EnsureDistancesLen();
            for (int i = 0; i < rBot.Count; i++) latestDistances[idx++] = rBot[i].dist;
        }
        for (; idx < _beamCount; idx++) latestDistances[idx] = maxRange;

        // 2) (unchanged) kinematics for latestRow only — CSV uses only timestamp + props + distances
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

        // Fill latestRow for your agent (unchanged structure)
        int p = 0;
        latestRow[p++] = t;
        latestRow[p++] = yaw;   latestRow[p++] = pitch; latestRow[p++] = roll;
        latestRow[p++] = vel.x; latestRow[p++] = vel.y; latestRow[p++] = vel.z;
        latestRow[p++] = acc.x; latestRow[p++] = acc.y; latestRow[p++] = acc.z;

        for (int m = 0; m < motorCount; m++)
            latestRow[p++] = (m < motorStrength.Length ? motorStrength[m] : 0f);

        for (int i2 = 0; i2 < _beamCount; i2++)
            latestRow[p++] = latestDistances[i2];

        // 3) CSV row: Timestamp + Prop 1..6 + distances
        if (updateCsv && _writer != null)
        {
            var row = new List<string>(1 + motorCount + _beamCount) { t.ToString("F3", inv) };

            for (int m = 0; m < motorCount; m++)
                row.Add((m < motorStrength.Length ? motorStrength[m] : 0f).ToString("F3", inv));

            for (int i3 = 0; i3 < _beamCount; i3++)
            {
                float d = latestDistances[i3];
                if (d > maxRange - 1e-4f && !writeMaxRangeOnMiss)
                    row.Add(""); // blank for miss
                else
                    row.Add(d.ToString("F4", inv));
            }
            _writer.WriteLine(string.Join(",", row));
            _writer.Flush();
        }

        _first = false;
        _prevTimestamp = t;
        _prevPos = dronePos;
        _prevVel = vel;
    }

    void EnsureDistancesLen()
    {
        if (latestDistances == null || latestDistances.Length != _beamCount)
            latestDistances = new float[_beamCount];
    }

    void WriteDistancesIntoLatestRow()
    {
        if (latestRow == null || latestRow.Length != _rowLen)
            latestRow = new float[_rowLen];

        int start = HeaderFloatCount + motorCount;
        for (int i = 0; i < _beamCount; i++)
            latestRow[start + i] = latestDistances[i];
    }

    // --- pull motor strengths for the current tick ---
    void PullMotorStrengths()
    {
        // Prefer live controller if available
        if (motorController != null)
        {
            // Expect a method: float[] GetMotorStrengths()
            // (If your MotorController uses a different name, tell me and I’ll wire it.)
            try
            {
                var arr = motorController.GetMotorStrengths();
                if (arr != null && arr.Length > 0)
                {
                    for (int i = 0; i < Mathf.Min(motorCount, arr.Length); i++)
                        motorStrength[i] = Mathf.Clamp01(arr[i]);
                    return;
                }
            }
            catch { /* no such method or null: fall back below */ }
        }

        // Otherwise: keep whatever is already in motorStrength[] (can be filled by your own code)
        for (int i = 0; i < motorCount; i++)
            motorStrength[i] = (i < motorStrength.Length) ? Mathf.Clamp01(motorStrength[i]) : 0f;
    }
}

