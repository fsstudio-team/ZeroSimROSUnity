using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace ZO.Editor {

    [CustomEditor(typeof(ZO.ZOSimDocumentRoot))]
    public class ZOSimRootComponentEditor : UnityEditor.Editor {

        public override void OnInspectorGUI() {

            DrawDefaultInspector();

            ZO.ZOSimDocumentRoot zoSimBaseComponent = (ZOSimDocumentRoot)target;

            if (GUILayout.Button("Save ZOSim")) {
                Debug.Log("INFO: ZOSimBaseComponent Save ZOSim");
                zoSimBaseComponent.SaveToZOSimFile();
            }

            if (GUILayout.Button("Load ZOSim")) {
                Debug.Log("INFO: ZOSimBaseComponent Load ZOSim");
                zoSimBaseComponent.LoadFromZOSimFile(zoSimBaseComponent.ZOSimDocumentFilePath);
            }

        }
    }
}