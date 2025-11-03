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

    // clamp deltaTime to avoid extreme jumps
    private float GetStableDeltaTime()
    {
        return Mathf.Clamp(Time.deltaTime, 0f, 0.05f); // max ~20 FPS
    }

    // called once per frame
    void Update()
    {
        float dt = GetStableDeltaTime();

        // go through each car in the list
        foreach (var carPath in cars)
        {
            // skip if no car or no waypoints assigned
            if (carPath.car == null || carPath.waypoints.Length == 0) continue;

            // current target waypoint
            Transform target = carPath.waypoints[carPath.currentWaypoint];
            Vector3 dir = (target.position - carPath.car.position).normalized;

            // move car
            // carPath.car.position += dir * carPath.speed * dt;
            carPath.car.position = Vector3.MoveTowards(carPath.car.position, target.position, carPath.speed * dt);


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

    // call this at the start of each episode to reset cars
    public void ResetCars()
    {
        foreach (var carPath in cars)
        {
            if (carPath.car == null || carPath.waypoints.Length < 2) continue;

            carPath.currentWaypoint = 0;
            carPath.car.position = carPath.waypoints[0].position;
            Vector3 dir = (carPath.waypoints[1].position - carPath.waypoints[0].position).normalized;
            carPath.car.rotation = Quaternion.LookRotation(dir);
        }
    }
}
