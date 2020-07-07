using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace ZO.Editor {

    [CustomEditor(typeof(ZO.Controllers.ZO2FingerGripController))]
    public class ZO2FingerGripControllerEditor : UnityEditor.Editor {
        public override void OnInspectorGUI() {

            DrawDefaultInspector();

            ZO.Controllers.ZO2FingerGripController gripController = (ZO.Controllers.ZO2FingerGripController)target;

            ZO.Controllers.ZOServoMotorActuator[] servorMotorActuators = gripController.GetComponents<ZO.Controllers.ZOServoMotorActuator>();
            string[] servorMotorActuatorChoices = new string[servorMotorActuators.Length];
            int currentIndex0 = -1;
            int currentIndex1 = -1;
            for (int i = 0; i < servorMotorActuators.Length; i++) {
                servorMotorActuatorChoices[i] = servorMotorActuators[i].Name;
                if (servorMotorActuators[i] == gripController._fingerActuators[0]) {
                    currentIndex0 = i;
                }
                if (servorMotorActuators[i] == gripController._fingerActuators[1]) {
                    currentIndex1 = i;
                }
            }

            int chosenIndex0 = UnityEditor.EditorGUILayout.Popup("Servo Motor 1", currentIndex0, servorMotorActuatorChoices);
            int chosenIndex1 = UnityEditor.EditorGUILayout.Popup("Servo Motor 2", currentIndex1, servorMotorActuatorChoices);

            if (chosenIndex0 != currentIndex0) {
                gripController._fingerActuators[0] = servorMotorActuators[chosenIndex0];
            }

            if (chosenIndex1 != currentIndex1) {
                gripController._fingerActuators[1] = servorMotorActuators[chosenIndex1];
            }

        }
    }

}
