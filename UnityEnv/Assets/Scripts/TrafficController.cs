using UnityEngine;
using System.Collections.Generic;

[System.Serializable]
public class CarPath
{
    public Transform car; // the car object (static mesh)
    public Transform[] waypoints; // positions along the path
    public float speed = 5f; // movement speed
    [HideInInspector] public int currentWaypoint = 0;
}

public class TrafficController : MonoBehaviour
{
    public CarPath[] cars;

    void Update()
    {
        foreach (var carPath in cars)
        {
            if (carPath.car == null || carPath.waypoints.Length == 0) continue;

            // current target waypoint
            Transform target = carPath.waypoints[carPath.currentWaypoint];
            Vector3 dir = (target.position - carPath.car.position).normalized;

            // move car
            carPath.car.position += dir * carPath.speed * Time.deltaTime;

            // check if reached waypoint
            float dist = Vector3.Distance(carPath.car.position, target.position);
            if (dist < 0.1f)
            {
                carPath.currentWaypoint = (carPath.currentWaypoint + 1) % carPath.waypoints.Length;
            }

            // make car face direction of travel
            if (dir != Vector3.zero)
                carPath.car.rotation = Quaternion.LookRotation(dir);
        }
    }
}
