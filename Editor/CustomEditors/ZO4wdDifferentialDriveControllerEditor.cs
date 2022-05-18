using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using ZO.ROS.Controllers;
using ZO.Physics;


namespace ZO.Editor {

    [CustomEditor(typeof(ZO4wdDifferentialDriveController))]
    public class ZO4wdDifferentialDriveControllerEditor : UnityEditor.Editor {

        // int _hingeChoiceIndex = 0;
        string[] _motorChoiceNames;
        SerializedObject _diffDriveSerializedObject;
        SerializedProperty _frontRightMotorProp;
        SerializedProperty _rearRightMotorProp;
        SerializedProperty _frontLeftMotorProp;
        SerializedProperty _rearLeftMotorProp;

        private void OnEnable() {
            _diffDriveSerializedObject = new SerializedObject(target);
            _frontRightMotorProp = _diffDriveSerializedObject.FindProperty("_frontRightWheelMotor");
            _rearRightMotorProp = _diffDriveSerializedObject.FindProperty("_rearRightWheelMotor");
            _frontLeftMotorProp = _diffDriveSerializedObject.FindProperty("_frontLeftWheelMotor");
            _rearLeftMotorProp = _diffDriveSerializedObject.FindProperty("_rearLeftWheelMotor");
        }
        public override void OnInspectorGUI() {
            _diffDriveSerializedObject.Update();

            DrawDefaultInspector();

            ZO4wdDifferentialDriveController diffDrive = (ZO4wdDifferentialDriveController)target;

            // get the hinge joints on this object
            ZOHingeJoint[] hingeJoints = diffDrive.GetComponentsInChildren<ZOHingeJoint>();
            _motorChoiceNames = new string[hingeJoints.Length];

            // check if we already have set the hinge joints
            int frontRightWheelIndex = -1;
            int rearRightWheelIndex = -1;
            int frontLeftWheelIndex = -1;
            int rearLeftWheelIndex = -1;
            for (int i = 0; i < hingeJoints.Length; i++) {
                _motorChoiceNames[i] = hingeJoints[i].Name;
                if (hingeJoints[i] == diffDrive.FrontRightWheelMotor) {
                    frontRightWheelIndex = i;
                }
                if (hingeJoints[i] == diffDrive.RearRightWheelMotor) {
                    rearRightWheelIndex = i;
                }
                if (hingeJoints[i] == diffDrive.FrontLeftWheelMotor) {
                    frontLeftWheelIndex = i;
                }
                if (hingeJoints[i] == diffDrive.RearLeftWheelMotor) {
                    rearLeftWheelIndex = i;
                }

            }

            int frontLeftMotorChosenIndex = UnityEditor.EditorGUILayout.Popup("Front Left Wheel Motor Name", frontLeftWheelIndex, _motorChoiceNames);
            int frontRightMotorChosenIndex = UnityEditor.EditorGUILayout.Popup("Front Right Wheel Motor Name", frontRightWheelIndex, _motorChoiceNames);
            
            int rearLeftMotorChosenIndex = UnityEditor.EditorGUILayout.Popup("Rear Left Wheel Motor Name", rearLeftWheelIndex, _motorChoiceNames);
            int rearRightMotorChosenIndex = UnityEditor.EditorGUILayout.Popup("Rear Right Wheel Motor Name", rearRightWheelIndex, _motorChoiceNames);

            // if we chose something different then what is already selected then set it
            if (frontRightMotorChosenIndex != frontRightWheelIndex) {
                _frontRightMotorProp.objectReferenceValue = hingeJoints[frontRightMotorChosenIndex];
            }
            if (rearRightMotorChosenIndex != rearRightWheelIndex) {
                _rearRightMotorProp.objectReferenceValue = hingeJoints[rearRightMotorChosenIndex];
            }

            if (frontLeftMotorChosenIndex != frontLeftWheelIndex) {
                _frontLeftMotorProp.objectReferenceValue = hingeJoints[frontLeftMotorChosenIndex];
            }
            if (rearLeftMotorChosenIndex != rearLeftWheelIndex) {
                _rearLeftMotorProp.objectReferenceValue = hingeJoints[rearLeftMotorChosenIndex];
            }


            // apply the properties
            _diffDriveSerializedObject.ApplyModifiedProperties();

        }
    }

}
