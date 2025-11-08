//------------------------------------------------------------
// ATTENTION: This file is used to run various editor-time checks
// Feel free to delete or add any debug prints you need
//------------------------------------------------------------

using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine.Animations;

public class Debugger: MonoBehaviour
{
    [Tooltip("If true, disable suspect components so you can see if the mesh un-freezes.")]
    public bool autoDisableSuspects = true;

    [ContextMenu("Scan For Transform Writers")]
    void Awake()
    {
        // Limit to the visual subtree if you want:
        Transform visualRoot = transform.Find("Drone_rotated") ?? transform;

        foreach (var t in visualRoot.GetComponentsInChildren<Transform>(true))
        {
            if (t == transform) continue;

            // 1) Animator / Animation
            var anim = t.GetComponent<Animator>();
            if (anim)
            {
                Debug.LogWarning($"Animator found on {t.name}", t);
                if (autoDisableSuspects) anim.enabled = false;
            }
            var legacy = t.GetComponent<Animation>();
            if (legacy)
            {
                Debug.LogWarning($"Animation (legacy) found on {t.name}", t);
                if (autoDisableSuspects) legacy.enabled = false;
            }

            // 2) Constraints
            DisableIf<ParentConstraint>(t);
            DisableIf<PositionConstraint>(t);
            DisableIf<RotationConstraint>(t);
            DisableIf<ScaleConstraint>(t);
            DisableIf<AimConstraint>(t);
            DisableIf<LookAtConstraint>(t);

            // 3) Joints
            //DisableIf<Joint>(t); // catches Hinge/Fixed/Configurable/etc.

            // 4) Any script that might write the transform
            foreach (var b in t.GetComponents<MonoBehaviour>())
            {
                if (b == null) continue;
                // Safe heuristic: log anything except common renderers/colliders
                if (!(b is Renderer) && !(b is Collider))
                    Debug.Log($"Script on {t.name}: {b.GetType().Name}", t);
            }
        }
    }

    void DisableIf<T>(Transform t) where T : Behaviour
    {
        var c = t.GetComponent<T>();
        if (c)
        {
            Debug.LogWarning($"{typeof(T).Name} found on {t.name}", t);
            if (autoDisableSuspects) c.enabled = false;
        }
    }
}
