using UnityEngine;
using UnityEditor;
using System.Collections;
using System.IO;
using System.Text;
using System.Diagnostics;

namespace ZO.Export {
    class ZOConvexDecomposition : EditorWindow {

        [SerializeField] private string _selectedNames;
        [SerializeField] private string _dockerExePath = "/usr/bin/docker";
        [SerializeField] private bool _overwrite = true;
        [SerializeField] private float _scale = 1.0f;
        [SerializeField] private Vector3 _rotation = Vector3.zero;
        [SerializeField] private Vector3 _offset = Vector3.zero;

        [SerializeField] private int _resolution = 100000;
        [SerializeField] private int _maxhulls = 128;
        [SerializeField] private float _concavity = 0.0025f;
        [SerializeField] private int _planeDownsampling = 4;
        [SerializeField] private int _convexhullDownsampling = 4;
        [SerializeField] private float _alpha = 0.05f;
        [SerializeField] private float _beta = 0.05f;
        [SerializeField] private float _gamma = 0.00125f;
        [SerializeField] private float _delta = 0.05f;
        [SerializeField] private bool _pca = false;
        [SerializeField] private bool _mode = false;
        [SerializeField] private int _maxNumVerticesPerCH = 128;
        [SerializeField] private float _minVolumePerCH = 0.0001f;
        [SerializeField] private bool _convexHullApproximation = true;
        [SerializeField] private static string _collisionMeshBasePath = "blah";

        private void OnEnable() {
            var data = EditorPrefs.GetString("ZOConvexDecomposition", JsonUtility.ToJson(this, false));
            JsonUtility.FromJsonOverwrite(data, this);
        }

        private void OnDisable() {
            var data = JsonUtility.ToJson(this, false);
            EditorPrefs.SetString("ZOConvexDecomposition", data);
        }

        [MenuItem("Zero Sim/Generate Collision Geometry...")]
        public static void DoConvexDecomposition() {
            if (Selection.gameObjects.Length == 0) {
                EditorUtility.DisplayDialog("ERROR: Select a prefab", "Please select a prefab to generate collision meshes.", "OK");
                return;
            }

            if (Selection.gameObjects[0].scene.rootCount != 0) {
                EditorUtility.DisplayDialog("ERROR: Select a prefab", "Please select a prefab to generate collision meshes.", "OK");
                return;
            }

            GameObject selectedGameObject = Selection.gameObjects[0];

            string prefabPath = AssetDatabase.GetAssetPath(selectedGameObject);
            string prefabBaseName = Selection.objects[0].name;
            string prefabDirectory = Path.GetDirectoryName(prefabPath);

            _collisionMeshBasePath = System.IO.Path.Combine(prefabDirectory, prefabBaseName + "_collision");


            ZOConvexDecomposition window = ScriptableObject.CreateInstance<ZOConvexDecomposition>();
            window.ShowUtility();

        }

        private void OnGUI() {

            _dockerExePath = EditorGUILayout.TextField("Docker Exe: ", _dockerExePath);
            _overwrite = EditorGUILayout.Toggle("Overwrite old collision", _overwrite);
            _scale = EditorGUILayout.FloatField("Mesh Scale: ", _scale);
            _rotation = EditorGUILayout.Vector3Field("Rotation: ", _rotation);
            _offset = EditorGUILayout.Vector3Field("Offset: ", _offset);

            foreach (var t in Selection.objects) {
                _selectedNames += t.name + " ";
            }

            EditorGUILayout.LabelField("Convex Decomposition Parameters:");
            _resolution = EditorGUILayout.IntField("Max number of voxels:", _resolution);
            _maxhulls = EditorGUILayout.IntField("Max number convex hulls: ", _maxhulls);
            _concavity = EditorGUILayout.FloatField("Max allowed concavity:", _concavity);
            _planeDownsampling = EditorGUILayout.IntField("Granularity of clip plane:", _planeDownsampling);
            _convexhullDownsampling = EditorGUILayout.IntField("Convex hull down sampling:", _convexhullDownsampling);
            _alpha = EditorGUILayout.FloatField("Symmetry plane bias: ", _alpha);
            _beta = EditorGUILayout.FloatField("Revolution axis bias:", _beta);
            _gamma = EditorGUILayout.FloatField("Max concavity: ", _gamma);
            _delta = EditorGUILayout.FloatField("Local concavity bias:", _delta);
            _pca = EditorGUILayout.Toggle("Normalize mesh:", _pca);
            _mode = EditorGUILayout.Toggle("Voxel ~or~ Tetrahedon mode: ", _mode);
            _maxNumVerticesPerCH = EditorGUILayout.IntField("Max triangles:", _maxNumVerticesPerCH);
            _minVolumePerCH = EditorGUILayout.FloatField("Min volume: ", _minVolumePerCH);
            _convexHullApproximation = EditorGUILayout.Toggle("Do Convex Hull Approximation", _convexHullApproximation);


            EditorGUILayout.LabelField("Selected Objects: ", _selectedNames);

            _selectedNames = "";

            GameObject selectedGameObject = Selection.gameObjects[0];

            string prefabPath = AssetDatabase.GetAssetPath(selectedGameObject);
            string prefabBaseName = Selection.objects[0].name;
            string prefabDirectory = Path.GetDirectoryName(prefabPath);

            if (_collisionMeshBasePath == "") {
                _collisionMeshBasePath = System.IO.Path.Combine(prefabDirectory, prefabBaseName + "_collision");
            }
            _collisionMeshBasePath = EditorGUILayout.TextField("Collision Mesh Base Path: ", _collisionMeshBasePath);

            EditorGUI.BeginDisabledGroup(Selection.objects.Length == 0);
            if (GUILayout.Button("Generate Convex Collision")) {

                // ~~~~ Create temporary .OBJ file to be later used as the base for the convex decomposition ~~~~ //
                string tempOBJFile = FileUtil.GetUniqueTempPathInProject() + ".obj";//Path.GetFullPath(FileUtil.GetUniqueTempPathInProject());

                UnityEngine.Debug.Log("INFO: Generate Convex Collision: Export OBJ File: " + tempOBJFile);
                UnityEngine.Debug.Log("INFO: Collision Mesh Base Path: " + _collisionMeshBasePath);

                ZOExportOBJ.DoExport(false, tempOBJFile);

                // ~~~~ Run Docker Convex Decomposition Python Script ~~~~ //

                // Note: All dockerized tools will have access to /unity-project-root as a volume
                string tempConvexMeshOBJDirectory = "Temp/collide-" + Path.GetFileNameWithoutExtension(tempOBJFile);
                string arguments = 
                              " --input_obj=/unity-project-root/" + tempOBJFile
                            + " --output_directory=/unity-project-root/" + tempConvexMeshOBJDirectory
                            + " --resolution=" + _resolution.ToString()
                            + " --maxhulls=" + _maxhulls.ToString()
                            + " --concavity=" + _concavity.ToString()
                            + " --planeDownsampling=" + _planeDownsampling.ToString()
                            + " --convexhullDownsampling=" + _convexhullDownsampling.ToString()
                            + " --alpha=" + _alpha.ToString()
                            + " --beta=" + _beta.ToString()
                            + " --gamma=" + _gamma.ToString()
                            + " --delta=" + _delta.ToString()
                            + " --pca=" + (_pca == true ? "1" : "0")
                            + " --mode=" + (_mode == true ? "1" : "0")
                            + " --maxNumVerticesPerCH=" + _maxNumVerticesPerCH.ToString()
                            + " --minVolumePerCH=" + _minVolumePerCH.ToString()
                            + " --convexhullApproximation=" + (_convexHullApproximation == true ? "1" : "0");
                
                UnityEngine.Debug.Log("INFO: Docker Args: " + arguments);

                string command = "conda run -n zosim_tools python /zo-asset-tools/zo_convex_decomposition/zo_convex_decomposition.py";
                string commandAWithArgs = $"{command} {arguments}";

                ZO.Editor.ZODockerManager.DockerRun(service: "zosim_tools", commandAWithArgs, null, (exitCode) => {

                    if(exitCode != 0){
                        UnityEngine.Debug.LogError($"Docker command error exit code: {exitCode}");
                        return;
                    }

                    UnityEngine.Debug.Log("Docker command Success.");

                    // ~~~~ Overwrite old collision meshes ~~~~ //
                    // GameObject prefab = null;
                    // if (_overwrite == true) {
                    //     Object[] assets = AssetDatabase.LoadAllAssetsAtPath(prefabPath);

                    //     foreach (var item in assets) {
                    //         if (item is Mesh) { // assign collider
                    //             if (item.name.Contains("-collision-")) {
                    //                 Object.DestroyImmediate(item, true);
                    //             }
                    //         }
                    //     }

                    //     prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
                    //     MeshCollider[] meshColliders = prefab.GetComponents<MeshCollider>();
                    //     foreach (var meshCollider in meshColliders) {
                    //         Object.DestroyImmediate(meshCollider, true);
                    //     }
                    //     PrefabUtility.SavePrefabAsset(prefab);
                    // }

                    // ~~~~ Create Collision Prefab Asset ~~~~ //
                    AssetDatabase.DeleteAsset(_collisionMeshBasePath);

                    // ~~~~ Import the convex collision meshes ~~~~ //
                    string[] convexMeshFiles = Directory.GetFiles(tempConvexMeshOBJDirectory);
                    int index = 1;
                    foreach (string sourceConveMeshOBJPath in convexMeshFiles) {

                        // Mesh mesh = ZO.Import.ZOImportOBJ.ImportFile(conveMeshOBJFile);
                        string meshName = _collisionMeshBasePath + index.ToString() + ".obj";
                        string rootProjectDirectory = Application.dataPath.Substring(0, Application.dataPath.LastIndexOf("/Assets"));
                        string destinationMeshAssetPath = Path.Combine(rootProjectDirectory, meshName); 
                        System.IO.File.Copy(sourceConveMeshOBJPath, destinationMeshAssetPath);

                        // force asset database refresh
                        UnityEditor.AssetDatabase.SaveAssets();
                        UnityEditor.AssetDatabase.Refresh();
                        Mesh mesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>(destinationMeshAssetPath);

                        UnityEngine.Debug.Log("INFO: Processing convex hull: " + meshName);

                        // // scale & rotate mesh
                        // Vector3[] meshVertices = mesh.vertices;
                        // Vector3[] transformedVertices = new Vector3[meshVertices.Length];
                        // Quaternion rotation = Quaternion.Euler(_rotation);
                        // for (int i = 0; i < meshVertices.Length; i++) {
                        //     Vector3 vertex = meshVertices[i];
                        //     vertex = rotation * vertex;
                        //     vertex.x = vertex.x * _scale;
                        //     vertex.y = vertex.y * _scale;
                        //     vertex.z = vertex.z * _scale;
                        //     vertex = vertex + _offset;

                        //     transformedVertices[i] = vertex;
                        // }
                        // mesh.vertices = transformedVertices;
                        // mesh.Optimize();
                        // mesh.RecalculateNormals();
                        // mesh.RecalculateBounds();

                        // AssetDatabase.CreateAsset(mesh, mesh.name + ".mesh");
                        // AssetDatabase.SaveAssets();

                        index++;

                    }

                    // NOTE: reloading prefab to get its changes
                    // Object[] objects = AssetDatabase.LoadAllAssetsAtPath(prefabPath);
                    // prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
                    // foreach (var item in objects) {
                    //     if (item is Mesh) { // assign collider
                    //         if (item.name.Contains("-collision-")) {
                    //             Mesh mesh = item as Mesh;
                    //             MeshCollider meshCollider = prefab.AddComponent<MeshCollider>();
                    //             meshCollider.sharedMesh = mesh;
                    //             meshCollider.convex = true;

                    //         }
                    //     }
                    // }


                    // // ~~~~ Finally Save It Back to the original prefab ~~~~ //
                    // PrefabUtility.SavePrefabAsset(prefab);

                    Close();

                });

                
            }
            EditorGUI.EndDisabledGroup();

        }

    }
}