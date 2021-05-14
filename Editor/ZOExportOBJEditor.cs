
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.IO;
using System.Text;
using ZO.ImportExport;

namespace ZO.Export {

    public class ZOExportOBJEditor : EditorWindow {
        public bool _exportSubMeshes = true;
        public bool _zeroPosition = true;
        private string _selectedNames;

        [MenuItem("Zero Sim/Export OBJ...")]
        public static void DoOBJExport() {
            //EditorWindow.GetWindow<ZOExportOBJ>();
            ZOExportOBJEditor window = ScriptableObject.CreateInstance<ZOExportOBJEditor>();
            window.ShowUtility();

        }

        private void OnGUI() {
            _exportSubMeshes = EditorGUILayout.Toggle("Export Sub Meshes: ", _exportSubMeshes);
            _zeroPosition = EditorGUILayout.Toggle("Zero Position: ", _zeroPosition);

            foreach (var t in Selection.objects) {
                _selectedNames += t.name + " ";
            }
            EditorGUILayout.LabelField("Selected Objects: ", _selectedNames);

            _selectedNames = "";

            EditorGUI.BeginDisabledGroup(Selection.gameObjects.Length == 0);
            if (GUILayout.Button("Export OBJ")) {
                string meshName = Selection.gameObjects[0].name;
                string fileDirectory = EditorUtility.OpenFolderPanel("Export .OBJ to directory", "", "");

                DoExport(_exportSubMeshes, fileDirectory);
            }
            EditorGUI.EndDisabledGroup();

        }

        private void OnInspectorUpdate() {
            Repaint();
        }

        public static void DoExport(bool makeSubmeshes, string directoryPath, bool zeroPosition = true, GameObject gameObject = null) {

            if (gameObject == null && Selection.gameObjects.Length == 0) {
                Debug.Log("ERROR: Didn't Export Any Meshes; Nothing was selected!");
                return;
            }

            if (gameObject == null) {
                gameObject = Selection.gameObjects[0];
            }
            ZOExportOBJ exportOBJ = new ZOExportOBJ();
            exportOBJ.ExportToDirectory(gameObject, directoryPath, makeSubmeshes, zeroPosition);

            // string meshName = gameObject.name;

            // ZOExportOBJ.Start();

            // StringBuilder meshString = new StringBuilder();

            // meshString.Append("#" + meshName + ".obj"
            //                   + "\n#" + System.DateTime.Now.ToLongDateString()
            //                   + "\n#" + System.DateTime.Now.ToLongTimeString()
            //                   + "\n#-------"
            //                   + "\n\n");

            // meshString.AppendLine($"mtllib {meshName}.mtl");

            // Transform transform = gameObject.transform;

            // Vector3 originalPosition = transform.position;
            // if (zeroPosition == true) {
            //     transform.position = Vector3.zero;
            // }

            // if (!makeSubmeshes) {
            //     meshString.Append("g ").Append(transform.name).Append("\n");
            // }
            // meshString.Append(ProcessTransform(transform, makeSubmeshes));


            // string objFilePath = Path.Combine(directoryPath, $"{meshName}.obj");

            // WriteToFile(meshString.ToString(), objFilePath);

            // transform.position = originalPosition;


            // // export materials and textures
            // StringBuilder mtlFileString = new StringBuilder();
            // HashSet<string> materialNames = new HashSet<string>();
            // MeshFilter meshFilter = transform.GetComponent<MeshFilter>();
            // if (meshFilter != null) {
            //     Material[] materials = meshFilter.GetComponent<Renderer>().sharedMaterials;
            //     foreach (Material material in materials) {
            //         if (materialNames.Contains(material.name) == false) {
            //             string materialString = ZOExportOBJ.MaterialToString(material);
            //             mtlFileString.Append(materialString);
            //             materialNames.Add(material.name);

            //             // handle texture
            //             if (material.HasProperty("_MainTex")) {
            //                 string assetPath = AssetDatabase.GetAssetPath(material.GetTexture("_MainTex"));
            //                 if (string.IsNullOrEmpty(assetPath) == false) {
            //                     string texName = Path.GetFileName(assetPath);
            //                     File.Copy(assetPath, Path.Combine(directoryPath, texName));
            //                 }
            //             }
            //         }
            //     }
            // }


            // // go through the children materials
            // for (int i = 0; i < transform.childCount; i++) {
            //     Transform child = transform.GetChild(i);
            //     meshFilter = child.GetComponent<MeshFilter>();

            //     if (meshFilter != null) {
            //         // meshString.Append(ZOObjExporterScript.MeshToString(meshFilter, transform));
            //         Material[] materials = meshFilter.GetComponent<Renderer>().sharedMaterials;
            //         foreach (Material material in materials) {
            //             if (materialNames.Contains(material.name) == false) {
            //                 string materialString = ZOExportOBJ.MaterialToString(material);
            //                 mtlFileString.Append(materialString);
            //                 materialNames.Add(material.name);
            //                 // handle texture
            //                 if (material.HasProperty("_MainTex")) {
            //                     string assetPath = AssetDatabase.GetAssetPath(material.GetTexture("_MainTex"));
            //                     if (string.IsNullOrEmpty(assetPath) == false) {
            //                         string texName = Path.GetFileName(assetPath);
            //                         File.Copy(assetPath, Path.Combine(directoryPath, texName));
            //                     }
            //                 }
            //             }
            //         }
            //     }
            // }
            // string mtlFilePath = Path.Combine(directoryPath, $"{meshName}.mtl");
            // WriteToFile(mtlFileString.ToString(), mtlFilePath);

            // ZOExportOBJ.End();
            // Debug.Log("Exported Mesh: " + objFilePath);
        }

        // static string ProcessTransform(Transform transform, bool makeSubmeshes) {
        //     StringBuilder meshString = new StringBuilder();

        //     meshString.Append("#" + transform.name
        //                       + "\n#-------"
        //                       + "\n");

        //     if (makeSubmeshes) {
        //         meshString.Append("g ").Append(transform.name).Append("\n");
        //     }

        //     MeshFilter meshFilter = transform.GetComponent<MeshFilter>();
        //     if (meshFilter != null) {
        //         meshString.Append(ZOExportOBJ.MeshToString(meshFilter, transform));
        //     }

        //     for (int i = 0; i < transform.childCount; i++) {
        //         meshString.Append(ProcessTransform(transform.GetChild(i), makeSubmeshes));
        //     }

        //     return meshString.ToString();
        // }

        // static void WriteToFile(string s, string filename) {
        //     using (StreamWriter sw = new StreamWriter(filename)) {
        //         sw.Write(s);
        //     }
        // }
    }
}