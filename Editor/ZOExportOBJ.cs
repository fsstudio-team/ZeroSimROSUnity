
using UnityEngine;
using UnityEditor;
using System.Collections;
using System.IO;
using System.Text;

namespace ZO.Export {
    public class ZOObjExporterScript {
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
            Mesh m = meshFilter.sharedMesh;
            if (!m) {
                return "####Error####";
            }
            Material[] mats = meshFilter.GetComponent<Renderer>().sharedMaterials;

            StringBuilder sb = new StringBuilder();

            foreach (Vector3 vv in m.vertices) {
                Vector3 v = transform.TransformPoint(vv);
                numVertices++;
                sb.Append(string.Format("v {0} {1} {2}\n", v.x, v.y, -v.z));
            }
            sb.Append("\n");
            foreach (Vector3 nn in m.normals) {
                Vector3 v = r * nn;
                sb.Append(string.Format("vn {0} {1} {2}\n", -v.x, -v.y, v.z));
            }
            // sb.Append("\n");
            // foreach (Vector3 v in m.uv) {
            //     sb.Append(string.Format("vt {0} {1}\n", v.x, v.y));
            // }
            
            for (int material = 0; material < m.subMeshCount; material++) {
                sb.Append("\n");
                //sb.Append("usemtl ").Append(mats[material].name).Append("\n");
                //sb.Append("usemap ").Append(mats[material].name).Append("\n");

                int[] triangles = m.GetTriangles(material);
                for (int i = 0; i < triangles.Length; i += 3) {
                    // sb.Append(string.Format("f {0}/{0}/{0} {1}/{1}/{1} {2}/{2}/{2}\n",
                    //                     triangles[i] + 1 + _startIndex, 
                    //                     triangles[i + 1] + 1 + _startIndex, 
                    //                     triangles[i + 2] + 1 + _startIndex));
                    sb.Append(string.Format("f {0}//{0} {1}//{1} {2}//{2}\n", // pos, None, Norm
                                        triangles[i] + 1 + _startIndex, 
                                        triangles[i + 1] + 1 + _startIndex, 
                                        triangles[i + 2] + 1 + _startIndex));

                }
            }

            _startIndex += numVertices;
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
                string fileName = EditorUtility.SaveFilePanel("Export .obj file", "", meshName, "obj");

                DoExport(_exportSubMeshes, fileName);
            }
            EditorGUI.EndDisabledGroup();

        }

        private void OnInspectorUpdate() {
            Repaint();
        }

        public static void DoExport(bool makeSubmeshes, string fileName, bool zeroPosition=true) {
            if (Selection.gameObjects.Length == 0) {
                Debug.Log("Didn't Export Any Meshes; Nothing was selected!");
                return;
            }

            string meshName = Selection.gameObjects[0].name;
            // string fileName = EditorUtility.SaveFilePanel("Export .obj file", "", meshName, "obj");

            ZOObjExporterScript.Start();

            StringBuilder meshString = new StringBuilder();

            meshString.Append("#" + meshName + ".obj"
                              + "\n#" + System.DateTime.Now.ToLongDateString()
                              + "\n#" + System.DateTime.Now.ToLongTimeString()
                              + "\n#-------"
                              + "\n\n");

            Transform transform = Selection.gameObjects[0].transform;

            Vector3 originalPosition = transform.position;
            if (zeroPosition == true) {                
                transform.position = Vector3.zero;
            }

            if (!makeSubmeshes) {
                meshString.Append("g ").Append(transform.name).Append("\n");
            }
            meshString.Append(ProcessTransform(transform, makeSubmeshes));

            WriteToFile(meshString.ToString(), fileName);

            transform.position = originalPosition;

            ZOObjExporterScript.End();
            Debug.Log("Exported Mesh: " + fileName);
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
                meshString.Append(ZOObjExporterScript.MeshToString(meshFilter, transform));
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