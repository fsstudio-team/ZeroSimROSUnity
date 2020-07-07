using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace ZO.Editor {
    [CustomEditor(typeof(ZO.Physics.ZOHingeJoint))]

    public class ZOHingeJointEditor : UnityEditor.Editor {
        public override void OnInspectorGUI() {
            DrawDefaultInspector();

            // ZO.Physics.ZOHingeJoint hingeJoint = (ZO.Physics.ZOHingeJoint)target;

            // UnityEngine.HingeJoint unityHinge = hingeJoint.UnityHinge;

            // serializedObject.Update();
            // SerializedProperty serializedProperty = serializedObject.FindProperty("_hinge");
            // EditorGUILayout.PropertyField(serializedProperty);
            // serializedObject.ApplyModifiedProperties();
        }
    }

}

