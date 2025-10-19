using UnityEngine;

public class BirdController : MonoBehaviour
{
    [System.Serializable]
    public class Bird
    {
        public Transform birdObject;
        public float speed = 5f;
        // public float turnSpeed = 2f;
    // turnSpeed is degrees per second now for RotateTowards
    public float turnSpeed = 120f;
    // how quickly the bird accelerates toward desired velocity
    public float acceleration = 5f;
        public float changeTargetInterval = 5f;

        [HideInInspector] public Vector3 targetPosition;
        [HideInInspector] public float timer;
        [HideInInspector] public Vector3 currentVelocity;
    }

    public Bird[] birds;
    public Vector3 areaSize = new Vector3(50f, 20f, 50f);
    public Vector3 areaCenter = Vector3.zero;

    void Start()
    {
        foreach (var bird in birds)
        {
            SetNewTarget(bird);
        }
    }

    void Update()
    {
        foreach (var bird in birds)
        {
            // compute desired velocity toward the target position
            Vector3 toTarget = bird.targetPosition - bird.birdObject.position;
            Vector3 desiredDir = toTarget.normalized;
            Vector3 desiredVelocity = desiredDir * bird.speed;

            // accelerate current velocity toward desired velocity
            bird.currentVelocity = Vector3.MoveTowards(
                bird.currentVelocity,
                desiredVelocity,
                bird.acceleration * Time.deltaTime
            );

            // integrate position
            bird.birdObject.position += bird.currentVelocity * Time.deltaTime;

            // rotate smoothly toward movement direction when moving
            if (bird.currentVelocity.sqrMagnitude > 0.01f)
            {
                Quaternion targetRotation = Quaternion.LookRotation(bird.currentVelocity.normalized);
                bird.birdObject.rotation = Quaternion.RotateTowards(
                    bird.birdObject.rotation,
                    targetRotation,
                    bird.turnSpeed * Time.deltaTime
                );
            }

            // update timer to pick new target occasionally
            bird.timer += Time.deltaTime;
            if (bird.timer >= bird.changeTargetInterval ||
                Vector3.Distance(bird.birdObject.position, bird.targetPosition) < 1f)
            {
                SetNewTarget(bird);
                bird.timer = 0f;
            }
        }
    }

    void SetNewTarget(Bird bird)
    {
        Vector3 randomOffset = new Vector3(
            Random.Range(-areaSize.x / 2, areaSize.x / 2),
            Random.Range(-areaSize.y / 2, areaSize.y / 2),
            Random.Range(-areaSize.z / 2, areaSize.z / 2)
        );
        bird.targetPosition = areaCenter + randomOffset;
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireCube(areaCenter, areaSize);
    }
}