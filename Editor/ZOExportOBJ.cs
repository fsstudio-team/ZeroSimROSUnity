
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace ZO.Export {
    public class ZOObjExporter {
        private static int _startIndex = 0;

        public static void Start() {
            _startIndex = 0;
        }
        public static void End() {
            _startIndex = 0;
        }


        public static string MeshToString(MeshFilter meshFilter, Transform transform) {
            Vector3 s = transform.localScale;
            Vector3 p = transform.localPosition;
            Quaternion r = transform.localRotation;


            int numVertices = 0;
            Mesh mesh = meshFilter.sharedMesh;
            if (!mesh) {
                return "####Error####";
            }
            Material[] mats = meshFilter.GetComponent<Renderer>().sharedMaterials;

            StringBuilder sb = new StringBuilder();

            foreach (Vector3 vv in mesh.vertices) {
                Vector3 v = transform.TransformPoint(vv);
                numVertices++;
                sb.Append(string.Format("v {0} {1} {2}\n", v.x, v.y, -v.z));
            }
            sb.Append("\n");
            foreach (Vector3 nn in mesh.normals) {
                Vector3 v = r * nn;
                sb.Append(string.Format("vn {0} {1} {2}\n", -v.x, -v.y, v.z));
            }
            sb.Append("\n");
            foreach (Vector3 v in mesh.uv) {
                sb.Append(string.Format("vt {0} {1}\n", v.x, v.y));
            }

            for (int material = 0; material < mesh.subMeshCount; material++) {
                sb.Append("\n");
                sb.Append("usemtl ").Append(mats[material].name).Append("\n");
                sb.Append("usemap ").Append(mats[material].name).Append("\n");

                int[] triangles = mesh.GetTriangles(material);
                for (int i = 0; i < triangles.Length; i += 3) {
                    sb.Append(string.Format("f {0}/{0}/{0} {1}/{1}/{1} {2}/{2}/{2}\n",
                                        triangles[i] + 1 + _startIndex,
                                        triangles[i + 1] + 1 + _startIndex,
                                        triangles[i + 2] + 1 + _startIndex));
                    // sb.Append(string.Format("f {0}//{0} {1}//{1} {2}//{2}\n", // pos, None, Norm
                    //                     triangles[i] + 1 + _startIndex, 
                    //                     triangles[i + 1] + 1 + _startIndex, 
                    //                     triangles[i + 2] + 1 + _startIndex));

                }
            }

            _startIndex += numVertices;
            return sb.ToString();
        }

        public static string MaterialToString(Material material) {
            StringBuilder sb = new StringBuilder();
            sb.AppendFormat("newmtl {0}", material.name).AppendLine();
            if (material.HasProperty("_Color")) {
                Color c = material.GetColor("_Color");
                sb.AppendFormat("Kd {0} {1} {2}", c.r, c.g, c.b).AppendLine();
            }
            if (material.HasProperty("_MainTex")) {
                string assetPath = AssetDatabase.GetAssetPath(material.GetTexture("_MainTex"));
                string texName = Path.GetFileName(assetPath);
                // string exportPath = Path.Combine(dir, texName);
                sb.AppendFormat("map_Kd {0}", texName).AppendLine();
                // if (!File.Exists(exportPath))
                // {
                //     File.Copy(assetPath, exportPath);
                // }
            }
            return sb.ToString();
        }
    }

    public class ZOExportOBJ : EditorWindow {
        public bool _exportSubMeshes = false;
        public bool _zeroPosition = true;
        private string _selectedNames;

        [MenuItem("Zero Sim/Export OBJ...")]
        public static void DoOBJExport() {
            //EditorWindow.GetWindow<ZOExportOBJ>();
            ZOExportOBJ window = ScriptableObject.CreateInstance<ZOExportOBJ>();
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

        public static void DoExport(bool makeSubmeshes, string fileDirectory, bool zeroPosition = true, GameObject gameObject = null) {

            if (gameObject == null && Selection.gameObjects.Length == 0) {
                Debug.Log("Didn't Export Any Meshes; Nothing was selected!");
                return;
            }

            if (gameObject == null) {
                gameObject = Selection.gameObjects[0];
            }

            string meshName = gameObject.name;
            // string fileName = EditorUtility.SaveFilePanel("Export .obj file", "", meshName, "obj");

            ZOObjExporter.Start();

            StringBuilder meshString = new StringBuilder();

            meshString.Append("#" + meshName + ".obj"
                              + "\n#" + System.DateTime.Now.ToLongDateString()
                              + "\n#" + System.DateTime.Now.ToLongTimeString()
                              + "\n#-------"
                              + "\n\n");

            meshString.AppendLine($"mtllib {meshName}.mtl");

            Transform transform = gameObject.transform;

            Vector3 originalPosition = transform.position;
            if (zeroPosition == true) {
                transform.position = Vector3.zero;
            }

            if (!makeSubmeshes) {
                meshString.Append("g ").Append(transform.name).Append("\n");
            }
            meshString.Append(ProcessTransform(transform, makeSubmeshes));


            string objFilePath = Path.Combine(fileDirectory, $"{meshName}.obj");

            WriteToFile(meshString.ToString(), objFilePath);

            transform.position = originalPosition;


            // export materials and textures
            StringBuilder mtlFileString = new StringBuilder();
            HashSet<string> materialNames = new HashSet<string>();
            MeshFilter meshFilter = transform.GetComponent<MeshFilter>();
            if (meshFilter != null) {
                Material[] materials = meshFilter.GetComponent<Renderer>().sharedMaterials;
                foreach (Material material in materials) {
                    if (materialNames.Contains(material.name) == false) {
                        string materialString = ZOObjExporter.MaterialToString(material);
                        mtlFileString.Append(materialString);
                        materialNames.Add(material.name);
                    }
                }
            }


            // go through the children materials
            for (int i = 0; i < transform.childCount; i++) {
                Transform child = transform.GetChild(i);
                meshFilter = child.GetComponent<MeshFilter>();

                if (meshFilter != null) {
                    // meshString.Append(ZOObjExporterScript.MeshToString(meshFilter, transform));
                    Material[] materials = meshFilter.GetComponent<Renderer>().sharedMaterials;
                    foreach (Material material in materials) {
                        if (materialNames.Contains(material.name) == false) {
                            string materialString = ZOObjExporter.MaterialToString(material);
                            mtlFileString.Append(materialString);
                            materialNames.Add(material.name);
                        }
                    }
                }
            }
            string mtlFilePath = Path.Combine(fileDirectory, $"{meshName}.mtl");
            WriteToFile(mtlFileString.ToString(), mtlFilePath);

            ZOObjExporter.End();
            Debug.Log("Exported Mesh: " + objFilePath);
        }

        static string ProcessTransform(Transform transform, bool makeSubmeshes) {
            StringBuilder meshString = new StringBuilder();

            meshString.Append("#" + transform.name
                              + "\n#-------"
                              + "\n");

            if (makeSubmeshes) {
                meshString.Append("g ").Append(transform.name).Append("\n");
            }

            MeshFilter meshFilter = transform.GetComponent<MeshFilter>();
            if (meshFilter != null) {
                meshString.Append(ZOObjExporter.MeshToString(meshFilter, transform));
            }

            for (int i = 0; i < transform.childCount; i++) {
                meshString.Append(ProcessTransform(transform.GetChild(i), makeSubmeshes));
            }

            return meshString.ToString();
        }

        static void WriteToFile(string s, string filename) {
            using (StreamWriter sw = new StreamWriter(filename)) {
                sw.Write(s);
            }
        }
    }
}