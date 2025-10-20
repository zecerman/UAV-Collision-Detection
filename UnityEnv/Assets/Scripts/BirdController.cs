using UnityEngine;

public class BirdController : MonoBehaviour
{
    // each bird will use this class
    [System.Serializable]
    public class Bird
    {
        public Transform birdObject; // bird object in the scene
        public float speed = 1f; // movement speed
        public float turnSpeed = 1f; // how quickly it turns toward target
        public float changeTargetInterval = 5f; // how often to pick a new target

        [HideInInspector] public Vector3 targetPosition; // current target position
        [HideInInspector] public float timer; // counts time to switch targets
        [HideInInspector] public Vector3 currentVelocity; // smooth movement
    }

    // list of all birds to control
    public Bird[] birds;
    // defines the 3d area the birds can fly in
    public Vector3 areaSize = new Vector3(50f, 20f, 50f);
    public Vector3 areaCenter = Vector3.zero;

    // called once when the scene starts
    void Start()
    {
        // give each bird a random starting target to fly forward
        foreach (var bird in birds)
        {
            SetNewTarget(bird);
        }
    }

    // called every frame
    void Update()
    {
        // loop through every bird and update its movement
        foreach (var bird in birds)
        {
            // use damping to smoothly move the bird, gradually changes position
            bird.birdObject.position = Vector3.SmoothDamp(
                bird.birdObject.position,
                bird.targetPosition,
                ref bird.currentVelocity,
                0.8f / bird.speed // smaller denominator = smoother motion
            );

            // calculate desired direction
            Vector3 direction = bird.currentVelocity.normalized;
            if (direction.magnitude > 0.1f)
            {
                // smoothly rotate toward movement direction
                Quaternion targetRotation = Quaternion.LookRotation(direction);
                bird.birdObject.rotation = Quaternion.Slerp(
                    bird.birdObject.rotation,
                    targetRotation,
                    Time.deltaTime * bird.turnSpeed
                );
            }

            // update timer to pick new target
            bird.timer += Time.deltaTime;

            // if enough time has passed, or the bird reached its target, pick a new one
            if (bird.timer >= bird.changeTargetInterval ||
                Vector3.Distance(bird.birdObject.position, bird.targetPosition) < 1f)
            {
                SetNewTarget(bird); // pick a new random target
                bird.timer = 0f; // reset timer
            }
        }
    }

    // chooses a new random point in the area for the bird to go to
    void SetNewTarget(Bird bird)
    {
        Vector3 randomOffset = new Vector3(
            Random.Range(-areaSize.x / 2, areaSize.x / 2),
            Random.Range(-areaSize.y / 2, areaSize.y / 2),
            Random.Range(-areaSize.z / 2, areaSize.z / 2)
        );
        // set the target position to a random point in the area
        bird.targetPosition = areaCenter + randomOffset;
    }

    // visualize the bird area in unity
    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireCube(areaCenter, areaSize);
    }
}
