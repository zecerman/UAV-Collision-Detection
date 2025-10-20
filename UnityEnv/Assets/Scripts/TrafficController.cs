using UnityEngine;

// defines one car and its path
[System.Serializable] // makes this show up in the Inspector
public class CarPath
{
    public Transform car; // the car object (static mesh)
    public Transform[] waypoints; // positions along the path
    public float speed = 5f; // movement speed
    // keeps track of which waypoint the car is currently moving toward
    [HideInInspector] public int currentWaypoint = 0;
}

// moves multiple cars along their own waypoint paths
public class TrafficController : MonoBehaviour
{
    public CarPath[] cars; // a list of all cars and their paths

    // called once per frame
    void Update()
    {
        // go through each car in the list
        foreach (var carPath in cars)
        {
            // skip if no car or no waypoints assigned
            if (carPath.car == null || carPath.waypoints.Length == 0) continue;

            // current target waypoint
            Transform target = carPath.waypoints[carPath.currentWaypoint];
            Vector3 dir = (target.position - carPath.car.position).normalized;

            // move car
            carPath.car.position += dir * carPath.speed * Time.deltaTime;

            // check if reached waypoint
            float dist = Vector3.Distance(carPath.car.position, target.position);
            // if the car is close enough to the waypoint, move to the next one
            // the "%" loops back to the first waypoint after the last one
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
