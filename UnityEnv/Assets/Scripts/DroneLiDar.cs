using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class DroneLiDAR : MonoBehaviour
{
    [Header("LiDAR parameters")]
    public int horizontalSamples = 36;        // how many rays around
    public float maxRange = 10f;              // maximum ray distance
    public float minRange = 0.1f;             // minimum
    public float scanInterval = 0.5f;         // time between scans in seconds

    [Header("Visualization")]
    public Material lineMaterial;             // assign a simple material in Inspector
    public float lineWidth = 0.02f;

    [Header("Output")]
    public string outputFileName = "LiDAR_Scan.csv";

    private string filePath;
    private List<LineRenderer> lines = new List<LineRenderer>();
    private float nextScanTime = 0f;

    void Start()
    {
        // File path
        filePath = Path.Combine(Application.dataPath, outputFileName);

        // Write header row
        using (var sw = new StreamWriter(filePath, false))
        {
            List<string> headers = new List<string>();
            headers.Add("T");
            for (int i = 0; i < horizontalSamples; i++)
            {
                headers.Add($"{i + 1}.x");
                headers.Add($"{i + 1}.y");
                headers.Add($"{i + 1}.z");
            }
            sw.WriteLine(string.Join(",", headers));
        }

        Debug.Log("LiDAR CSV will be saved at: " + filePath);

        // Create LineRenderers once
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
    }

    void Update()
    {
        // Run scan every scanInterval seconds
        if (Time.time >= nextScanTime)
        {
            DoLiDARScan();
            nextScanTime = Time.time + scanInterval;
        }
    }

    void DoLiDARScan()
    {
        float timestamp = Time.time;   // actual Unity elapsed time

        List<string> row = new List<string>();
        row.Add(timestamp.ToString("F3"));

        for (int i = 0; i < horizontalSamples; i++)
        {
            float angle = ((float)i / horizontalSamples) * 360f;
            Quaternion rot = Quaternion.Euler(0f, angle, 0f);
            Vector3 dir = rot * transform.forward;

            Ray ray = new Ray(transform.position, dir);
            RaycastHit hit;
            Vector3 hitPos = transform.position + dir * maxRange;

            if (Physics.Raycast(ray, out hit, maxRange))
            {
                hitPos = hit.point;
            }

            // Update line renderer
            LineRenderer lr = lines[i];
            lr.SetPosition(0, transform.position);
            lr.SetPosition(1, hitPos);

            // Add XYZ
            row.Add(hitPos.x.ToString("F3"));
            row.Add(hitPos.y.ToString("F3"));
            row.Add(hitPos.z.ToString("F3"));
        }

        // Append row
        using (var sw = new StreamWriter(filePath, true))
        {
            sw.WriteLine(string.Join(",", row));
        }

        Debug.Log("LiDAR scan at " + timestamp.ToString("F2") + "s");
    }
}

