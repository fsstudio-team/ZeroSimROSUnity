using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace ZO.Editor {
    [CustomEditor(typeof(ZO.Physics.ZOHingeJoint)), CanEditMultipleObjects]

    public class ZOHingeJointEditor : UnityEditor.Editor {

        private static readonly string[] _dontIncludeMe = new string[] { "_updateRateHz" };

        public override void OnInspectorGUI() {
            DrawPropertiesExcluding(serializedObject, _dontIncludeMe);
            serializedObject.ApplyModifiedProperties();

            // DrawDefaultInspector();

            // ZO.Physics.ZOHingeJoint hingeJoint = (ZO.Physics.ZOHingeJoint)target;

            // UnityEngine.HingeJoint unityHinge = hingeJoint.UnityHinge;

            // serializedObject.Update();
            // SerializedProperty serializedProperty = serializedObject.FindProperty("_hinge");
            // EditorGUILayout.PropertyField(serializedProperty);
            // serializedObject.ApplyModifiedProperties();
        }
    }

}

