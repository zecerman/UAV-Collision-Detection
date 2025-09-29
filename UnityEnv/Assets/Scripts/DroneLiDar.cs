using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class DroneLidar : MonoBehaviour
{
    [Header("LiDAR parameters")]
    public int horizontalSamples = 36;        // how many rays around
    public float maxRange = 10f;              // maximum ray distance
    public float minRange = 0.1f;              // minimum
    public float scanInterval = 0.5f;         // time between scans in seconds

    [Header("Visualization")]
    public Material lineMaterial;      // assign a simple material in Inspector
    public float lineWidth = 0.02f;

    [Header("Output")]
    public string outputFileName = "LiDAR_Scan.csv";

    [ContextMenu("Reset CSV")]
    public void ResetCSV()
{
    using (var writer = new StreamWriter(filePath, false))
    {
        writer.WriteLine("timestamp, angleDeg, distance");
    }
    Debug.Log("CSV reset: " + filePath);
}

    private float scanTimer = 0f;
    private string filePath;

    // Store line renderers so we can reuse them
    private List<LineRenderer> lines = new List<LineRenderer>();

    void Start()
    {
        // csv file for LiDar Data
        filePath = Path.Combine(Application.dataPath, outputFileName);

        // Header Row
        using (var writer = new StreamWriter(filePath, false))  // overwrite
        {
            writer.WriteLine("timestamp, angleDeg, distance");
        }

        // LineRenderers once
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
        scanTimer += Time.deltaTime;
        if (scanTimer >= scanInterval)
        {
            DoLiDARScan();
            scanTimer = 0f;
        }
    }

    void DoLiDARScan()
    {
        float timestamp = Time.time;

        for (int i = 0; i < horizontalSamples; i++)
        {
            // compute ray direction
            float angle = ((float)i / horizontalSamples) * 360f;
            Quaternion rot = Quaternion.Euler(0f, angle, 0f);
            Vector3 dir = rot * transform.forward;

            Ray ray = new Ray(transform.position, dir);
            RaycastHit hit;
            float distance = maxRange;
            Vector3 endPos = transform.position + dir * maxRange;

            if (Physics.Raycast(ray, out hit, maxRange))
            {
                distance = Mathf.Max(hit.distance, minRange);
                endPos = hit.point;
            }

            // Updates LineRenderer
            LineRenderer lr = lines[i];
            lr.SetPosition(0, transform.position);
            lr.SetPosition(1, endPos);

            // Writes to CSV
            using (var writer = new StreamWriter(filePath, true))
            {
                writer.WriteLine($"{timestamp}, {angle}, {distance}");
            }
        }
    }
}
