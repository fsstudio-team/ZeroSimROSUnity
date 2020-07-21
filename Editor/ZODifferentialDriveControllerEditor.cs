using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using ZO.Controllers;
using ZO.Physics;


namespace ZO.Editor {

    [CustomEditor(typeof(ZODifferentialDriveController))]
    public class ZODifferentialDriveControllerEditor : UnityEditor.Editor {

        // int _hingeChoiceIndex = 0;
        string[] _motorChoiceNames;
        SerializedObject _diffDriveSerializedObject;
        SerializedProperty _rightMotorProp;
        SerializedProperty _leftMotorProp;

        private void OnEnable() {
            _diffDriveSerializedObject = new SerializedObject(target);
            _rightMotorProp = _diffDriveSerializedObject.FindProperty("_rightWheelMotor");
            _leftMotorProp = _diffDriveSerializedObject.FindProperty("_leftWheelMotor");
        }
        public override void OnInspectorGUI() {
            _diffDriveSerializedObject.Update();

            DrawDefaultInspector();



            ZODifferentialDriveController diffDrive = (ZODifferentialDriveController)target;

            // get the hinge joints on this object
            ZOHingeJoint[] hingeJoints = diffDrive.GetComponents<ZOHingeJoint>();
            _motorChoiceNames = new string[hingeJoints.Length];

            // check if we already have set the hinge joints
            int rightWheelIndex = -1;
            int leftWheelIndex = -1;
            for (int i = 0; i < hingeJoints.Length; i++) {
                _motorChoiceNames[i] = hingeJoints[i].Name;
                if (hingeJoints[i] == diffDrive.RightWheelMotor) {
                    rightWheelIndex = i;
                }
                if (hingeJoints[i] == diffDrive.LeftWheelMotor) {
                    leftWheelIndex = i;
                }

            }

            int rightMotorChosenIndex = UnityEditor.EditorGUILayout.Popup("Right Wheel Motor Name", rightWheelIndex, _motorChoiceNames);

            int leftMotorChosenIndex = UnityEditor.EditorGUILayout.Popup("Left Wheel Motor Name", leftWheelIndex, _motorChoiceNames);


            // if we chose something different then what is already selected then set it
            if (rightMotorChosenIndex != rightWheelIndex) {
                _rightMotorProp.objectReferenceValue = hingeJoints[rightMotorChosenIndex];
            }

            if (leftMotorChosenIndex != leftWheelIndex) {
                _leftMotorProp.objectReferenceValue = hingeJoints[leftMotorChosenIndex];
            }


            // apply the properties
            _diffDriveSerializedObject.ApplyModifiedProperties();



        }
    }

}
