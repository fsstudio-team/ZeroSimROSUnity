using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using ZO.Document;


namespace ZO.Editor {

    [CustomEditor(typeof(ZOSimDocumentRoot))]
    public class ZOSimDocumentRootEditor : UnityEditor.Editor {

        public override void OnInspectorGUI() {

            DrawDefaultInspector();

            ZOSimDocumentRoot zoSimBaseComponent = (ZOSimDocumentRoot)target;

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