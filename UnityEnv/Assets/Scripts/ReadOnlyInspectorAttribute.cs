using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

// This attribute marks a field as read-only in the Unity Inspector
public class ReadOnlyInspectorAttribute : PropertyAttribute { }

#if UNITY_EDITOR
// This drawer defines how the ReadOnlyInspectorAttribute is drawn in the Inspector
[CustomPropertyDrawer(typeof(ReadOnlyInspectorAttribute))]
public class ReadOnlyInspectorDrawer : PropertyDrawer
{
    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
    {
        GUI.enabled = false;  // disable editing
        EditorGUI.PropertyField(position, property, label, true);
        GUI.enabled = true;   // re-enable editing for other fields
    }
}
#endif
