using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace ZO.Editor {

    [CustomEditor(typeof(ZO.Controllers.ZOServoMotorActuator))]
    public class ZOServerMotorActuatorEditor : UnityEditor.Editor {

        // int _hingeChoiceIndex = 0;
        string[] _hingeJointChoices;
        public override void OnInspectorGUI() {

            DrawDefaultInspector();

            ZO.Controllers.ZOServoMotorActuator servoMotorActuator = (ZO.Controllers.ZOServoMotorActuator)target;

            ZO.Physics.ZOHingeJoint[] hingeJoints = servoMotorActuator.GetComponents<ZO.Physics.ZOHingeJoint>();
            _hingeJointChoices = new string[hingeJoints.Length];
            int currentIndex = -1;
            for (int i = 0; i < hingeJoints.Length; i++) {
                _hingeJointChoices[i] = hingeJoints[i].Name;
                if (hingeJoints[i] == servoMotorActuator._hingeJoint) {
                    currentIndex = i;
                }
                
            }

            int chosenIndex = UnityEditor.EditorGUILayout.Popup("ZO Hinge Joint", currentIndex, _hingeJointChoices);

            if (chosenIndex != currentIndex) {
                servoMotorActuator._hingeJoint = hingeJoints[chosenIndex];
                SerializedProperty hingeJointProp = serializedObject.FindProperty("_hingeJoint");
                hingeJointProp.objectReferenceValue = hingeJoints[chosenIndex];
                serializedObject.ApplyModifiedProperties();

            }
            
            
        }
    }

}
