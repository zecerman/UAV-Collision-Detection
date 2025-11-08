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
    // Schema: [ t, yaw, pitch, roll, vx, vy, vz, ax, ay, az, motor1..motor6, beam_distances... ]
    public float[] latestRow;            // ALWAYS initialized at Start()
    public float[] latestDistances;      // distances only, length = beamCount (convenience)

    [Header("Motors")]
    public int motorCount = 6;
    public float[] motorStrength = new float[6];

    [Header("Shared Ray Params")]
    public float maxRange = 30f;
    public float minRange = 0.05f;

    [Header("Shared Raycast Settings (applied to all sensors)")]
    public LayerMask environmentLayers = ~0;  // include ground/obstacles; exclude Drone
    public QueryTriggerInteraction triggerMode = QueryTriggerInteraction.Ignore;

    [Header("CSV Logging (disable during training)")]
    public bool enableCsvLogging = false;
    public string outputFileBase = "LiDAR_Scan";
    private string _sessionPath;
    private StreamWriter _writer;

    // --- internals ---
    float _nextScanTime;
    float _prevTimestamp = -1f;
    Vector3 _prevPos, _prevVel;
    bool _first = true;

    // fixed-column header portion counts
    const int HeaderFloatCount = 10;  // t, yaw, pitch, roll, vx, vy, vz, ax, ay, az
    int _beamCount = 0;               // computed at Start from sensors
    int _rowLen = 0;                  // HeaderFloatCount + motorCount + _beamCount

void Awake()
{
    if (motorStrength == null || motorStrength.Length < motorCount)
        motorStrength = new float[motorCount];

    // Auto-find sensors if needed
    if (!topSensor)   { var t = GameObject.Find("LiDAR Sensor Top");    if (t) topSensor = t.GetComponent<LiDARSensor>(); }
    if (!bottomSensor){ var b = GameObject.Find("LiDAR Sensor Bottom"); if (b) bottomSensor = b.GetComponent<LiDARSensor>(); }

    // Ensure roots
    if (topSensor   && !topSensor.droneRoot)    topSensor.droneRoot    = transform;
    if (bottomSensor&& !bottomSensor.droneRoot) bottomSensor.droneRoot = transform;

    // Force beams now so counts are valid
    if (topSensor)    topSensor.RebuildBeams();
    if (bottomSensor) bottomSensor.RebuildBeams();

    // Compute & allocate
    _beamCount = (topSensor ? topSensor.BeamCount : 0) + (bottomSensor ? bottomSensor.BeamCount : 0);
    _rowLen = HeaderFloatCount + motorCount + _beamCount;

    latestDistances = new float[_beamCount];
    for (int i = 0; i < _beamCount; i++) latestDistances[i] = maxRange;

    latestRow = new float[_rowLen];
    WriteDistancesIntoLatestRow();

    // Seed one real scan immediately
    _prevPos = transform.position;
    _prevVel = Vector3.zero;
    DoScan(updateCsv: false);

    // Start schedule
    _nextScanTime = Time.time + scanInterval;

    Debug.Log($"[LiDARLogger] Awake: beams={_beamCount}, rowLen={_rowLen}");
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

        // Ensure sensors know their droneRoot
        if (topSensor  && !topSensor.droneRoot)    topSensor.droneRoot = transform;
        if (bottomSensor && !bottomSensor.droneRoot) bottomSensor.droneRoot = transform;

        // Force beams to be built now so BeamCount is valid on frame 0
        if (topSensor)    topSensor.RebuildBeams();
        if (bottomSensor) bottomSensor.RebuildBeams();

        // Compute constant beam count
        _beamCount = (topSensor ? topSensor.BeamCount : 0) + (bottomSensor ? bottomSensor.BeamCount : 0);
        _rowLen = HeaderFloatCount + motorCount + _beamCount;

        // Seed live arrays with valid lengths so Agent can read immediately
        latestRow = new float[_rowLen];
        latestDistances = new float[_beamCount];

        // Reasonable defaults before first true scan:
        //  - header floats & motors stay 0
        //  - distances initialized to maxRange (no hit)
        for (int i = 0; i < _beamCount; i++) latestDistances[i] = maxRange;
        WriteDistancesIntoLatestRow(); // copies latestDistances into tail of latestRow

        // Prepare CSV if enabled (write header once)
        if (enableCsvLogging)
        {
            var dir = Path.Combine(Application.dataPath, "LiDAR_Logs");
            Directory.CreateDirectory(dir);
            _sessionPath = Path.Combine(dir, $"{outputFileBase}_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");
            var fs = new FileStream(_sessionPath, FileMode.Create, FileAccess.Write, FileShare.Read);
            _writer = new StreamWriter(fs);
            WriteCsvHeader();
            _writer.Flush();
        }

        _prevPos = transform.position;
        _prevVel = Vector3.zero;

        // Do an immediate scan so latestRow is “real” before Agent’s first CollectObservations
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
    void OnDestroy()         { CloseWriter();
#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }

    void CloseWriter()
    {
        if (_writer != null) { _writer.Flush(); _writer.Dispose(); _writer = null; }
    }

    // ---------- CSV HEADER ----------
    void WriteCsvHeader()
    {
        var H = new List<string>
        {
            "Timestamp(s)",
            "yaw(deg)","pitch(deg)","roll(deg)",
            "vx(m_s)","vy(m_s)","vz(m_s)",
            "ax(m_s2)","ay(m_s2)","az(m_s2)"
        };
        for (int m = 1; m <= motorCount; m++) H.Add($"motor{m}.strength");

        for (int i = 0; i < _beamCount; i++)
        {
            int k = i + 1;
            H.Add($"beam{k}.dist(m)");
        }

        _writer.WriteLine(string.Join(",", H));
    }

    // ---------- SCAN ----------
    void DoScan(bool updateCsv)
    {
        var inv = CultureInfo.InvariantCulture;

        // 1) Scan beams (distances only for training)
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
        // If one sensor is null, idx may be < _beamCount; pad remaining with maxRange
        for (; idx < _beamCount; idx++) latestDistances[idx] = maxRange;

        // 2) Compute kinematics for header
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

        // 3) Fill latestRow (header + motors + distances), no allocations
        int p = 0;
        latestRow[p++] = t;
        latestRow[p++] = yaw;   latestRow[p++] = pitch; latestRow[p++] = roll;
        latestRow[p++] = vel.x; latestRow[p++] = vel.y; latestRow[p++] = vel.z;
        latestRow[p++] = acc.x; latestRow[p++] = acc.y; latestRow[p++] = acc.z;

        for (int m = 0; m < motorCount; m++)
            latestRow[p++] = (m < motorStrength.Length ? motorStrength[m] : 0f);

        // copy distances tail
        for (int i2 = 0; i2 < _beamCount; i2++)
            latestRow[p++] = latestDistances[i2];

        // 4) CSV (optional)
        if (updateCsv && _writer != null)
        {
            var row = new List<string>(_rowLen);
            int r = 0;
            // header floats
            for (int i3 = 0; i3 < HeaderFloatCount; i3++) row.Add(latestRow[r++].ToString(i3 < 1 ? "F3" : (i3 <= 3 ? "F3" : "F4"), inv));
            // motors
            for (int i3 = 0; i3 < motorCount; i3++) row.Add(latestRow[r++].ToString("F3", inv));
            // distances
            for (int i3 = 0; i3 < _beamCount; i3++) row.Add(latestRow[r++].ToString("F4", inv));

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
}
