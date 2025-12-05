using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

[DefaultExecutionOrder(-1000)] // ensure this .cs file runs first
[DisallowMultipleComponent]
public class LiDARLogger : MonoBehaviour
{

    [Header("Sensors")]
    public LiDARSensor topSensor;    // assign "LiDAR Sensor Top"
    public LiDARSensor bottomSensor; // assign "LiDAR Sensor Bottom"

    [Header("Goal (for distances)")]
    [Tooltip("Optional: reference to goal object so we can log x/y/z distance to it.")]
    public Transform goal;

    [Header("Timing")]
    [Tooltip("Seconds between scans")]
    public float scanInterval = 0.05f;

    [Header("Live Access (read by DroneAgent)")]
    // Schema:
    // [ t,
    //   yaw, pitch, roll,
    //   vx, vy, vz,
    //   ax, ay, az,
    //   x_distance, y_distance, z_distance,
    //   beam_distances... ]
    public float[] latestRow;            // ALWAYS initialized at Start()
    public float[] latestDistances;      // distances only, length = beamCount (convenience)

    [Header("Shared Raycast Settings (applied to all sensors)")]
    public LayerMask environmentLayers = ~0;  // include ground/obstacles; exclude Drone
    public QueryTriggerInteraction triggerMode = QueryTriggerInteraction.Ignore;

    [Header("CSV Logging (disable during training)")]
    public bool enableCsvLogging = false;
    public string outputFileBase = "LiDAR_Scan";
    private string _sessionPath;
    private StreamWriter _writer;

    [Header("Raycast Settings")]
    public float maxRange = 20f;
    public float minRange = 0.05f;
    public LayerMask hitLayers; 

    //  internals 
    float _nextScanTime;
    float _prevTimestamp = -1f;
    Vector3 _prevPos, _prevVel;
    bool _first = true;

    // fixed-column header portion counts
    // t, yaw, pitch, roll, vx,vy,vz, ax,ay,az, x_distance, y_distance, z_distance
    const int HeaderFloatCount = 13;
    int _beamCount = 0;               // computed at Start from sensors
    int _rowLen = 0;                  // HeaderFloatCount + _beamCount

    void Awake()
    {
        // Layer mask defaults
        if (hitLayers.value == 0)
        {
            // Try to exclude the "Drone" layer if it exists.
            int droneLayer = LayerMask.NameToLayer("Drone");
            if (droneLayer >= 0)
            {
                // Everything except Drone.
                hitLayers = ~(1 << droneLayer);
                Debug.Log($"[LiDARLogger] hitLayers not set; defaulting to ~Drone mask = 0x{hitLayers.value:X8}");
            }
            else
            {
                // Fallback: include Default
                hitLayers = LayerMask.GetMask("Default");
                if (hitLayers.value == 0)
                {
                    // Ultimate fallback: everything.
                    hitLayers = ~0;
                }
                Debug.Log($"[LiDARLogger] hitLayers not set; fallback mask = 0x{hitLayers.value:X8}");
            }
        }

        // Auto-find sensors if needed
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

        // Ensure roots
        if (topSensor   && !topSensor.droneRoot)    topSensor.droneRoot    = transform;
        if (bottomSensor&& !bottomSensor.droneRoot) bottomSensor.droneRoot = transform;

        // Make sure sensors use the same hitLayers / triggerMode as this logger
        if (topSensor)
        {
            topSensor.hitLayers          = hitLayers;
            topSensor.triggerInteraction = triggerMode;
            topSensor.RebuildBeams();
        }
        if (bottomSensor)
        {
            bottomSensor.hitLayers          = hitLayers;
            bottomSensor.triggerInteraction = triggerMode;
            bottomSensor.RebuildBeams();
        }

        // Compute & allocate
        _beamCount = (topSensor ? topSensor.BeamCount : 0) + (bottomSensor ? bottomSensor.BeamCount : 0);
        _rowLen = HeaderFloatCount + _beamCount;

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

        Debug.Log($"[LiDARLogger] Awake: beams={_beamCount}, rowLen={_rowLen}, hitLayers=0x{hitLayers.value:X8}");
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
        _rowLen = HeaderFloatCount + _beamCount;

        // Seed live arrays with valid lengths so Agent can read immediately
        latestRow = new float[_rowLen];
        latestDistances = new float[_beamCount];

        // Reasonable defaults before first true scan:
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

    // CSV HEADER
    void WriteCsvHeader()
    {
        var H = new List<string>
        {
            "Timestamp(s)",
            "yaw(deg)","pitch(deg)","roll(deg)",
            "vx(m_s)","vy(m_s)","vz(m_s)",
            "ax(m_s2)","ay(m_s2)","az(m_s2)",
            "x_distance(m)","y_distance(m)","z_distance(m)"  // NEW
        };

        for (int i = 0; i < _beamCount; i++)
        {
            int k = i + 1;
            H.Add($"beam{k}.dist(m)");
        }

        _writer.WriteLine(string.Join(",", H));
    }

    // SCAN
void DoScan(bool updateCsv)
{
    var inv = CultureInfo.InvariantCulture;

    int idx = 0;

    // 1) Scan beams (distances only for training)
    if (topSensor)
    {
        var rTop = topSensor.ScanOnce(maxRange, minRange, hitLayers, triggerMode);
        EnsureDistancesLen();
        for (int i = 0; i < rTop.Count; i++)
        {
            float d = rTop[i].dist;

            // Sanitize distance
            if (float.IsNaN(d) || float.IsInfinity(d) || d < 0f)
            {
                Debug.LogWarning($"[LiDARLogger] Invalid top distance {d} on beam {i}, clamping to maxRange");
                d = maxRange;
            }
            else
            {
                // Clamp within [minRange, maxRange]
                d = Mathf.Clamp(d, minRange, maxRange);
            }

            latestDistances[idx++] = d;
        }
    }

    if (bottomSensor)
    {
        var rBot = bottomSensor.ScanOnce(maxRange, minRange, hitLayers, triggerMode);
        EnsureDistancesLen();
        for (int i = 0; i < rBot.Count; i++)
        {
            float d = rBot[i].dist;

            if (float.IsNaN(d) || float.IsInfinity(d) || d < 0f)
            {
                Debug.LogWarning($"[LiDARLogger] Invalid bottom distance {d} on beam {i}, clamping to maxRange");
                d = maxRange;
            }
            else
            {
                d = Mathf.Clamp(d, minRange, maxRange);
            }

            latestDistances[idx++] = d;
        }
    }

    // Pad remaining beams (if one sensor missing, etc.)
    for (; idx < _beamCount; idx++)
        latestDistances[idx] = maxRange;

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

    // distance from drone to goal in world XYZ
    Vector3 dGoal = Vector3.zero;
    if (goal != null)
    {
        dGoal = goal.position - dronePos; // same sign as (goal - drone)
    }

    // 3) Fill latestRow (header + distances)
    int p = 0;
    latestRow[p++] = t;
    latestRow[p++] = yaw;   latestRow[p++] = pitch; latestRow[p++] = roll;
    latestRow[p++] = vel.x; latestRow[p++] = vel.y; latestRow[p++] = vel.z;
    latestRow[p++] = acc.x; latestRow[p++] = acc.y; latestRow[p++] = acc.z;
    latestRow[p++] = dGoal.x;  // x_distance
    latestRow[p++] = dGoal.y;  // y_distance
    latestRow[p++] = dGoal.z;  // z_distance

    for (int i2 = 0; i2 < _beamCount; i2++)
        latestRow[p++] = latestDistances[i2];

    // 4) CSV (optional)
    if (updateCsv && _writer != null)
    {
        var row = new List<string>(_rowLen);
        int r = 0;
        // header floats (0..12)
        for (int i3 = 0; i3 < HeaderFloatCount; i3++)
        {
            string fmt;
            if (i3 == 0)          fmt = "F3"; // time
            else if (i3 <= 3)     fmt = "F3"; // yaw/pitch/roll
            else                  fmt = "F4"; // velocities, acc, distances
            row.Add(latestRow[r++].ToString(fmt, inv));
        }
        // distances
        for (int i3 = 0; i3 < _beamCount; i3++)
            row.Add(latestRow[r++].ToString("F4", inv));

        _writer.WriteLine(string.Join(",", row));
        _writer.Flush();
    }

    _first = false;
    _prevTimestamp = t;
    _prevPos = dronePos;
    _prevVel = vel;
}


    // HELPERS
    void EnsureDistancesLen()
    {
        if (latestDistances == null || latestDistances.Length != _beamCount)
            latestDistances = new float[_beamCount];
    }

    void WriteDistancesIntoLatestRow()
    {
        if (latestRow == null || latestRow.Length != _rowLen)
            latestRow = new float[_rowLen];

        int start = HeaderFloatCount;
        for (int i = 0; i < _beamCount; i++)
            latestRow[start + i] = latestDistances[i];
    }
}
