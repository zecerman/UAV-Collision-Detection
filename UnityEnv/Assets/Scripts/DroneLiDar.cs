using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class DroneLiDAR : MonoBehaviour
{
    [Header("LiDAR parameters")]
    public int horizontalSamples = 36;
    public float maxRange = 10f;
    public float minRange = 0.1f;
    public float scanInterval = 0.5f;

    [Header("Visualization")]
    public Material lineMaterial;
    public float lineWidth = 0.02f;

    [Header("Output")]
    public string outputFileName = "LiDAR_Scan.csv";

    private string filePath;
    private List<LineRenderer> lines = new List<LineRenderer>();
    private float nextScanTime = 0f;

    // Fields for motion tracking ---
    private Vector3 prevPos;
    private Vector3 prevVel;
    private bool firstScan = true;

    void Start()
    {
        filePath = Path.Combine(Application.dataPath, outputFileName);

        // Header Row
        using (var sw = new StreamWriter(filePath, false))
        {
            List<string> headers = new List<string> {
                "Timestamp",
                "x.velocity", "y.velocity", "z.velocity",
                "x.acceleration", "y.acceleration", "z.acceleration"
            };

            for (int i = 0; i < horizontalSamples; i++)
            {
                int lidarNum = i + 1;
                headers.Add($"{lidarNum}.x");
                headers.Add($"{lidarNum}.y");
                headers.Add($"{lidarNum}.z");
                headers.Add($"{lidarNum}.euclidean_dist");
            }

            sw.WriteLine(string.Join(",", headers));
        }

        // Line renderers
        for (int i = 0; i < horizontalSamples; i++)
        {
            GameObject lineObj = new GameObject("LidarRay_" + i);
            lineObj.transform.parent = this.transform;

            LineRenderer lr = lineObj.AddComponent<LineRenderer>();
            lr.material = lineMaterial != null ? lineMaterial : new Material(Shader.Find("Sprites/Default"));
            lr.startWidth = lineWidth;
            lr.endWidth = lineWidth;
            lr.positionCount = 2;
            lr.startColor = Color.green;
            lr.endColor = Color.green;

            lines.Add(lr);
        }

        prevPos = transform.position;
        prevVel = Vector3.zero;
    }

    void Update()
    {
        if (Time.time >= nextScanTime)
        {
            DoLiDARScan();
            nextScanTime = Time.time + scanInterval;
        }
    }

    void DoLiDARScan()
    {
        float timestamp = Time.time;
        Vector3 currPos = transform.position;

        Vector3 velocity = Vector3.zero;
        Vector3 acceleration = Vector3.zero;

        if (!firstScan)
        {
            float dt = scanInterval;
            velocity = (currPos - prevPos) / dt;
            acceleration = (velocity - prevVel) / dt;
        }

        List<string> row = new List<string> {
            timestamp.ToString("F3"),
            velocity.x.ToString("F3"),
            velocity.y.ToString("F3"),
            velocity.z.ToString("F3"),
            acceleration.x.ToString("F3"),
            acceleration.y.ToString("F3"),
            acceleration.z.ToString("F3")
        };

        // LiDAR rays
        for (int i = 0; i < horizontalSamples; i++)
        {
            float angle = ((float)i / horizontalSamples) * 360f;
            Quaternion rot = Quaternion.Euler(0f, angle, 0f);
            Vector3 dir = rot * transform.forward;

            Ray ray = new Ray(currPos, dir);
            RaycastHit hit;
            Vector3 hitPos = currPos + dir * maxRange;

            if (Physics.Raycast(ray, out hit, maxRange))
                hitPos = hit.point;

            LineRenderer lr = lines[i];
            lr.SetPosition(0, currPos);
            lr.SetPosition(1, hitPos);

            float dist = Vector3.Distance(currPos, hitPos);

            row.Add(hitPos.x.ToString("F3"));
            row.Add(hitPos.y.ToString("F3"));
            row.Add(hitPos.z.ToString("F3"));
            row.Add(dist.ToString("F3"));
        }

        using (var sw = new StreamWriter(filePath, true))
            sw.WriteLine(string.Join(",", row));

        prevPos = currPos;
        prevVel = velocity;
        firstScan = false;
    }
}

