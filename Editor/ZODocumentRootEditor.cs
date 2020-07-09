using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace ZO.Editor {

    [CustomEditor(typeof(ZO.ZOSimDocumentRoot))]
    public class ZOSimDocumentRootEditor : UnityEditor.Editor {

        public override void OnInspectorGUI() {

            DrawDefaultInspector();

            ZO.ZOSimDocumentRoot zoSimBaseComponent = (ZOSimDocumentRoot)target;

            if (GUILayout.Button("Save ZOSim")) {
                Debug.Log("INFO: ZOSimBaseComponent Save ZOSim");
                if (EditorUtility.DisplayDialog("Override File:", "Override File: " + zoSimBaseComponent.ZOSimDocumentFilePath, "OK", "Cancel")) {
                    zoSimBaseComponent.SaveToZOSimFile(zoSimBaseComponent.ZOSimDocumentFilePath);
                }

            }

            if (GUILayout.Button("Load ZOSim")) {
                Debug.Log("INFO: ZOSimBaseComponent Load ZOSim");
                if (EditorUtility.DisplayDialog("Override Object:", "Override Object with file: " + zoSimBaseComponent.ZOSimDocumentFilePath, "OK", "Cancel")) {

                    zoSimBaseComponent.LoadFromZOSimFile(zoSimBaseComponent.ZOSimDocumentFilePath);
                }
            }

        }
    }
}