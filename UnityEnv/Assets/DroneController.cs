using UnityEngine;

public class DroneController : MonoBehaviour
{
    [System.Serializable]
    public class Rotor
    {
        public string name;
        public Transform transform;       // rotor position in world
        public KeyCode key;               // which arrow key controls it
        [HideInInspector] public float throttle; // 0..1, auto-ramped
    }

    public Rigidbody rb;
    public Rotor[] rotors = new Rotor[4];

    [Header("Thrust (Newtons)")]
    [Tooltip("Extra thrust per rotor when its key is fully held (on top of hover).")]
    public float extraThrustPerRotor = 15f;

    [Header("Spin-up / down (per second)")]
    public float spinUpRate = 2.5f;      // how quickly throttle rises toward 1
    public float spinDownRate = 3.5f;    // how quickly it falls toward 0

    [Header("Assist (optional)")]
    [Tooltip("Torque to gently keep the drone upright. Increase if it tips too easily.")]
    public float levelAssist = 2.0f;
    [Tooltip("Damps spin to reduce wobble.")]
    public float angularDamping = 0.25f;

    void Reset()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        if (!rb || rotors == null || rotors.Length == 0) return;

        float dt = Time.fixedDeltaTime;

        // 1) Update each rotor's throttle toward pressed/unpressed target
        foreach (var r in rotors)
        {
            bool pressed = Input.GetKey(r.key);
            float target = pressed ? 1f : 0f;
            float rate = pressed ? spinUpRate : spinDownRate;
            r.throttle = Mathf.MoveTowards(r.throttle, target, rate * dt);
        }

        // 2) Compute hover force per rotor to counteract gravity
        float hoverPerRotor = (rb.mass * Physics.gravity.magnitude) / rotors.Length;

        // 3) Apply force at each rotor's position
        foreach (var r in rotors)
        {
            // Upward (body up) thrust so it also tilts when forces are unbalanced
            float thrustN = hoverPerRotor + r.throttle * extraThrustPerRotor;
            rb.AddForceAtPosition(transform.up * thrustN, r.transform.position, ForceMode.Force);
        }

        // 4) Gentle auto-level + spin damping (optional but helps playability)
        Vector3 tilt = Vector3.Cross(transform.up, Vector3.up);            // torque toward upright
        Vector3 torque = (-tilt * levelAssist) + (-rb.angularVelocity * angularDamping);
        rb.AddTorque(torque, ForceMode.Acceleration);
    }
}
