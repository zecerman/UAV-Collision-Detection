using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;              
using System.Reflection;        
using UnityEngine;

[DisallowMultipleComponent]
public class LiDARLogger2 : MonoBehaviour
{
    [Header("Sensors")]
    public LiDARSensor2 frontSensor;
    public LiDARSensor2 backSensor;

    [Header("Timing")]
    public float scanInterval = 0.5f;

    [Header("Raycast Settings")]
    public LayerMask environmentLayers = ~0;
    public QueryTriggerInteraction triggerMode = QueryTriggerInteraction.Ignore;
    public float maxRange = 30f;
    public float minRange = 0.05f;

    [Header("Output Location")]
    public string logsFolderName = "LiDAR_Logs 2";
    public bool usePersistentPath = true;

    [Header("Miss Behavior")]
    [Tooltip("If true, write maxRange when a ray misses; if false, write empty cell.")]
    public bool writeMaxRangeOnMiss = true;

    // -------- MOTORS --------
    [Header("Motors")]
    [Tooltip("Optional: if assigned, motor strengths will be pulled from this controller each scan.")]
    public MotorController motorController;           

    [Tooltip("Fallback values if no controller is assigned (index 0..5).")]
    public float[] fallbackMotorStrength = new float[6];

    private readonly float[] _mBuf = new float[6];    // internal copy (always 6)

    // ---- internals ----
    private string _csvPath;
    private StreamWriter _writer;
    private float _nextScanTime;

    // cached header data
    private bool _headerWritten = false;
    private int  _beamCount = -1;
    private readonly List<float> _headerElev = new(); // degrees
    private readonly List<float> _headerAzim = new(); // degrees

    string GetLogsRoot()
    {
        var root = usePersistentPath ? Application.persistentDataPath : Application.dataPath;
        var full = Path.Combine(root, logsFolderName);
        Directory.CreateDirectory(full);
        return full;
    }

    void Start()
    {
        // optional auto-find
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

        // sync common settings
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

        StartNewCsv();
        _nextScanTime = Time.time;
    }

    void OnDestroy() { CloseWriter(); }
    void OnApplicationQuit() { CloseWriter(); }

    void CloseWriter()
    {
        if (_writer != null) { _writer.Flush(); _writer.Dispose(); _writer = null; }
    #if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
    #endif
    }

    void StartNewCsv()
    {
        CloseWriter();
        string dir = GetLogsRoot();
        _csvPath = Path.Combine(dir, $"LiDAR_Scan_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");
        var fs = new FileStream(_csvPath, FileMode.Create, FileAccess.Write, FileShare.Read);
        _writer = new StreamWriter(fs, new UTF8Encoding(encoderShouldEmitUTF8Identifier: true)); // UTF-8 BOM
        _headerWritten = false;
        _beamCount = -1;
        _headerElev.Clear();
        _headerAzim.Clear();
        Debug.Log($"LiDAR logging to: {_csvPath}");
    }

    void Update()
    {
        if (Time.time >= _nextScanTime)
        {
            DoScanAndWrite();
            _nextScanTime = Time.time + scanInterval;
        }
    }

    void DoScanAndWrite()
    {
        var inv = CultureInfo.InvariantCulture;

        // 1) scan both sensors
        List<LiDARSensor2.BeamResult> rFront = null, rBack = null;
        if (frontSensor)
            rFront = frontSensor.ScanOnce(frontSensor.maxRange, frontSensor.minRange, frontSensor.hitLayers, frontSensor.triggerInteraction);
        if (backSensor)
            rBack  = backSensor.ScanOnce (backSensor.maxRange,  backSensor.minRange,  backSensor.hitLayers,  backSensor.triggerInteraction);

        // 2) combine (front first, then back) for stable indexing
        var beams = new List<LiDARSensor2.BeamResult>((rFront?.Count ?? 0) + (rBack?.Count ?? 0));
        if (rFront != null) beams.AddRange(rFront);
        if (rBack  != null) beams.AddRange(rBack);

        int countNow = beams.Count;

        // 3) (Re)build header if needed
        if (!_headerWritten || _beamCount != countNow)
        {
            if (_headerWritten) StartNewCsv(); // start fresh file on beam layout change

            _beamCount = countNow;
            _headerElev.Clear();
            _headerAzim.Clear();

            // collect fixed angles from this first scan
            for (int i = 0; i < countNow; i++)
            {
                _headerElev.Add(beams[i].el);
                _headerAzim.Add(beams[i].az);
            }

            WriteHeader();
        }

        // 4) write row: Timestamp, Prop1..Prop6, then beam distances
        var row = new List<string>(1 + 6 + _beamCount) { Time.time.ToString("F3", inv) };

        FillMotorBuffer();
        for (int i = 0; i < 6; i++)
            row.Add(_mBuf[i].ToString("F3", inv));  // <-- added missing semicolon

        for (int i = 0; i < _beamCount; i++)
        {
            var br = beams[i];
            if (br.hit != 0)
                row.Add(br.dist.ToString("F4", inv));
            else
                row.Add(writeMaxRangeOnMiss ? maxRange.ToString("F4", inv) : ""); // blank or maxRange
        }

        _writer.WriteLine(string.Join(",", row));
        _writer.Flush();
    }

    void WriteHeader()
    {
        // Columns: Timestamp, Prop 1..Prop 6, then one column per beam (unchanged format)
        var H = new List<string>(1 + 6 + _beamCount) { "Timestamp(s)" };
        H.Add("Prop 1"); H.Add("Prop 2"); H.Add("Prop 3");
        H.Add("Prop 4"); H.Add("Prop 5"); H.Add("Prop 6");

        for (int i = 0; i < _beamCount; i++)
        {
            int k = i + 1;
            string label = $"beam{k} distance (elev={_headerElev[i]:F1}° | azim={_headerAzim[i]:F1}°)";
            H.Add(label); // no commas inside so columns don’t split
        }

        _writer.WriteLine(string.Join(",", H));
        _writer.Flush();
        _headerWritten = true;
        Debug.Log($"Header written with 6 motor columns + {_beamCount} beam columns.");
    }

    // --- helpers ---
    void FillMotorBuffer()
    {
        // zero default
        for (int i = 0; i < 6; i++) _mBuf[i] = 0f;

        if (motorController != null)
        {
            var t = motorController.GetType();
            const BindingFlags F = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;

            // 1) Method: float[] GetMotorStrengths()
            var mi = t.GetMethod("GetMotorStrengths", F);
            if (mi != null && mi.ReturnType == typeof(float[]) && mi.GetParameters().Length == 0)
            {
                var arr = (float[])mi.Invoke(motorController, null);
                if (arr != null)
                {
                    for (int i = 0; i < Mathf.Min(6, arr.Length); i++) _mBuf[i] = Mathf.Clamp01(arr[i]);
                    return;
                }
            }

            // 2) Field/Property: float[] motorStrength
            var fi = t.GetField("motorStrength", F);
            if (fi != null && typeof(float[]).IsAssignableFrom(fi.FieldType))
            {
                var arr = (float[])fi.GetValue(motorController);
                if (arr != null)
                {
                    for (int i = 0; i < Mathf.Min(6, arr.Length); i++) _mBuf[i] = Mathf.Clamp01(arr[i]);
                    return;
                }
            }
            var pi = t.GetProperty("motorStrength", F);
            if (pi != null && typeof(float[]).IsAssignableFrom(pi.PropertyType))
            {
                var arr = (float[])pi.GetValue(motorController, null);
                if (arr != null)
                {
                    for (int i = 0; i < Mathf.Min(6, arr.Length); i++) _mBuf[i] = Mathf.Clamp01(arr[i]);
                    return;
                }
            }

            // 3) Six scalars: prop1..prop6 (also try m1..m6, motor1..motor6), case-insensitive
            string[][] nameSets =
            {
                new[] { "prop1","prop2","prop3","prop4","prop5","prop6" },
                new[] { "m1","m2","m3","m4","m5","m6" },
                new[] { "motor1","motor2","motor3","motor4","motor5","motor6" }
            };

            for (int set = 0; set < nameSets.Length; set++)
            {
                bool allFound = true;
                for (int i = 0; i < 6; i++)
                {
                    string baseName = nameSets[set][i];

                    // try field then property; case-insensitive search
                    var f = t.GetField(baseName, F | BindingFlags.IgnoreCase);
                    if (f != null && f.FieldType == typeof(float))
                    {
                        _mBuf[i] = Mathf.Clamp01((float)f.GetValue(motorController));
                        continue;
                    }
                    var p = t.GetProperty(baseName, F | BindingFlags.IgnoreCase);
                    if (p != null && p.PropertyType == typeof(float))
                    {
                        _mBuf[i] = Mathf.Clamp01((float)p.GetValue(motorController, null));
                        continue;
                    }

                    allFound = false; break;
                }
                if (allFound) return;
            }
        }

        // 4) Fallback array on the logger itself
        if (fallbackMotorStrength != null)
            for (int i = 0; i < Mathf.Min(6, fallbackMotorStrength.Length); i++)
                _mBuf[i] = Mathf.Clamp01(fallbackMotorStrength[i]);
    }
} // <-- final closing brace for the class


