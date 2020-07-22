using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using UnityEngine;
using UnityEditor;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace ZO.Import {
    public class ZOImportZeroSim : EditorWindow {

        [SerializeField] public Vector3 _positionTransformScale = new Vector3(1, 1, 1);

        /// <summary>
        /// Scaling the actual mesh geometry. NOTE:  we do a -1 x scale because when
        /// Unity imports an OBJ it negates the x component so we are just reversing that.
        /// </summary>
        /// <returns></returns>
        [SerializeField] public Vector3 _meshTransformScale = new Vector3(-1, 1, 1);
        [SerializeField] public static bool _colorizeMeshPieces = true;

        static private string _zeroSimProjectFile = null;
        static public string ZeroSimProjectFile {
            get { return _zeroSimProjectFile; }
            set { _zeroSimProjectFile = value; }
        }

        private string DocumentName {
            get; set;
        }

        // the directory to export to
        private string ExportDirectoryPath {
            get; set;
        }

        // ExportDirectoryPath + document name
        private string RootExportDirectory {
            get { return Path.Combine(ExportDirectoryPath, DocumentName); }
        }

        // RootExportDirectory + "visual_meshes"
        private string VisualMeshDirectory {
            get { return Path.Combine(RootExportDirectory, "visual_meshes"); }
        }

        // RootExportDirectory + "collision_meshes"
        private string CollisionMeshDirectory {
            get { return Path.Combine(RootExportDirectory, "collision_meshes"); }
        }

        private string ImportDirectory {
            get { return Path.GetDirectoryName(ZeroSimProjectFile); }
        }

        private JObject ZeroSimJSON {
            get; set;
        }

        ZOSimDocumentRoot _documentRoot;

        [MenuItem("Zero Sim/Import ZeroSim Project...")]
        static public void ImportZeroSimProject() {
            Debug.Log("INFO: ZOImportZeroSim Start...");
            ZOImportZeroSim window = ScriptableObject.CreateInstance<ZOImportZeroSim>();
            window.position = new Rect(Screen.width / 2, Screen.height / 2, 250, 150);
            window.ShowPopup();
        }

        private void OnGUI() {
            _positionTransformScale = EditorGUILayout.Vector3Field("Position Transform Scale", _positionTransformScale);
            _meshTransformScale = EditorGUILayout.Vector3Field("Mesh Transform Scale", _meshTransformScale);
            _colorizeMeshPieces = EditorGUILayout.Toggle("Colorize Mesh Pieces: ", _colorizeMeshPieces);

            if (GUILayout.Button("Import ZeroSim Project")) {
                DoImport();
                this.Close();
            }

            if (GUILayout.Button("Cancel")) {
                this.Close();
            }
        }


        void DoImport() {
            Debug.Log("INFO: Zero Sim Import...");
            ZeroSimProjectFile = null;  // HACKHACK. Doing this because we don't want todo the "drag & drop" project into Unity.  Instead we always want to ask for the project file.
            if (ZeroSimProjectFile == null) {
                ZeroSimProjectFile = EditorUtility.OpenFilePanelWithFilters("Import Fusion 360 Project", "", new[] { "ZoSim", "zosim, zsim, json" });
            }
            Debug.Log("INFO: Zero Sim Import Project File: " + ZeroSimProjectFile);

            ZeroSimJSON = JObject.Parse(File.ReadAllText(ZeroSimProjectFile));
            DocumentName = ZeroSimJSON["document_name"].Value<string>();

            ExportDirectoryPath = EditorUtility.OpenFolderPanel("Select Save To Folder", Application.dataPath + "/Assets", "");

            // create directory structures            
            if (Directory.Exists(RootExportDirectory) == false) {
                Directory.CreateDirectory(RootExportDirectory);
            }
            if (Directory.Exists(VisualMeshDirectory) == false) {
                Directory.CreateDirectory(VisualMeshDirectory);
            }
            if (Directory.Exists(CollisionMeshDirectory) == false) {
                Directory.CreateDirectory(CollisionMeshDirectory);
            }


            // read all components and load any assets like visual and collision meshes
            foreach (JObject component in ZeroSimJSON["components"]) {
                OnComponent(component);
            }

            // set the scales that were set in the import UI
            // save out scales to JSON 
            ZeroSimJSON["position_transform_scale"] = new JArray(_positionTransformScale.x, _positionTransformScale.y, _positionTransformScale.z);
            ZeroSimJSON["mesh_transform_scale"] = new JArray(_meshTransformScale.x, _meshTransformScale.y, _meshTransformScale.z);

            // create the root game object that contains the ZOSimDocumentRoot
            GameObject rootGameObject = new GameObject(ZeroSimJSON["document_name"].Value<string>());

            // add the document root component
            _documentRoot = rootGameObject.AddComponent<ZO.ZOSimDocumentRoot>();
            string zosimSaveToFilePath = Path.Combine(ExportDirectoryPath, DocumentName + ".zosim");
            string zosimSaveToFilePathUnityRelative = MakeRelativePath(Application.dataPath, zosimSaveToFilePath);

            // deserialize the ZoSim JSON file
            _documentRoot.ZOSimDocumentFilePath = zosimSaveToFilePathUnityRelative;
            _documentRoot.Deserialize(ZeroSimJSON);



            // turn off self collisions
            rootGameObject.AddComponent<ZO.Util.ZOTurnOffSelfCollision>();

            // foreach (JObject occurrence in ZeroSimJSON["occurrences"]) {
            //     OnOccurrence(occurrence, rootGameObject, ZeroSimJSON);
            // }


            // save prefab 
            string prefabFilePath = ExportDirectoryPath + "/" + ZeroSimJSON["document_name"].Value<string>() + ".prefab";
            prefabFilePath = AssetDatabase.GenerateUniqueAssetPath(prefabFilePath);

            Debug.Log("INFO: Saving Prefab: " + prefabFilePath);

            PrefabUtility.SaveAsPrefabAsset(rootGameObject, prefabFilePath);


            // Save ZoSim file            
            File.WriteAllText(zosimSaveToFilePath, ZeroSimJSON.ToString());

            ZeroSimProjectFile = null; // need to reset

        }


        /// <summary>
        /// Loads assets from a ZoSim component such as "visual_mesh_file" and "collission_meshes"
        /// </summary>
        /// <param name="component"></param>
        private void OnComponent(JObject component) {
            // Copy visual meshes
            if (component.ContainsKey("visual_mesh_file") == true) {

                // copy the visual meshes over
                string source_visual_mesh_file_path = Path.Combine(ImportDirectory, component["visual_mesh_file"].Value<string>());
                string destination_visual_mesh_file_path = Path.Combine(VisualMeshDirectory, Path.GetFileName(component["visual_mesh_file"].Value<string>()));
                System.IO.File.Copy(source_visual_mesh_file_path, destination_visual_mesh_file_path, true);

                // force Unity to load the meshes
                UnityEditor.AssetDatabase.SaveAssets();
                UnityEditor.AssetDatabase.Refresh();

                string relative_visual_mesh_asset_path = "Assets" + destination_visual_mesh_file_path.Remove(0, Application.dataPath.Length);

                // UnityEngine.Object[] data = UnityEditor.AssetDatabase.LoadAllAssetsAtPath(relative_visual_mesh_asset_path);
                Mesh mesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>(relative_visual_mesh_asset_path);

                Debug.Log("INFO: loaded visual mesh: " + relative_visual_mesh_asset_path);

            }

            if (component.ContainsKey("collision_meshes") == true) {
                ZOImportZeroSimProcessor.IsProcessingZOSimColliders = true;
                foreach (string collisionMeshFile in component["collision_meshes"]) {
                    // copy the colllision meshes over
                    string sourceCollisionMeshFilePath = Path.Combine(ImportDirectory, "collision_meshes", collisionMeshFile);
                    string destinationCollisionMeshFilePath = Path.Combine(CollisionMeshDirectory, Path.GetFileName(collisionMeshFile));
                    System.IO.File.Copy(sourceCollisionMeshFilePath, destinationCollisionMeshFilePath, true);

                    // force Unity to load the meshes
                    UnityEditor.AssetDatabase.SaveAssets();
                    UnityEditor.AssetDatabase.Refresh();

                    // LoadAllAssetsAtPath must have a relative directory.... 
                    string relativeCollisionMeshAssetPath = "Assets" + destinationCollisionMeshFilePath.Remove(0, Application.dataPath.Length);
                    Mesh mesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>(relativeCollisionMeshAssetPath);


                    // relativeCollisionMeshAssetPath = Path.ChangeExtension(relativeCollisionMeshAssetPath, null);
                    // GameObject meshGameObjectParent = UnityEditor.AssetDatabase.LoadAssetAtPath<GameObject>(relativeCollisionMeshAssetPath);
                    // string assetPath = AssetDatabase.GetAssetPath(meshGameObjectParent);
                    // // GameObject meshGameObjectRoot = PrefabUtility.LoadPrefabContents(assetPath);
                    // PrefabAssetType prefabAssetType = PrefabUtility.GetPrefabAssetType(meshGameObjectParent);
                    // // BUGBUG: always assuming the 0 child game object 
                    // GameObject meshGameObject = meshGameObjectParent.transform.GetChild(0).gameObject;

                    // // create the meshcollider
                    // UnityEngine.MeshCollider meshCollider = meshGameObjectParent.AddComponent<UnityEngine.MeshCollider>();
                    // meshCollider.convex = true;

                    // // disable meshrenderer for collider
                    // meshGameObject.GetComponent<MeshRenderer>().enabled = false;

                    // // Save contents back to Prefab Asset and unload contents.
                    // PrefabUtility.SaveAsPrefabAsset(meshGameObjectParent, relativeCollisionMeshAssetPath);
                    // PrefabUtility.UnloadPrefabContents(meshGameObjectParent);

                    Debug.Log("INFO: loaded collision mesh: " + relativeCollisionMeshAssetPath);
                }

                ZOImportZeroSimProcessor.IsProcessingZOSimColliders = false;
            }
        }

        /*

        private void OnOccurrence(JObject occurrenceJson, GameObject parentOccurenceGo, JObject jsonDoc) {
            Debug.Log("INFO: processing occurrence: " + occurrenceJson["name"]);


            // create the occurrence GameObject
            GameObject occurenceGo = new GameObject(occurrenceJson["name"].Value<string>());
            occurenceGo.transform.parent = parentOccurenceGo.transform;

            ZOSimOccurrence zosimOccurence = occurenceGo.AddComponent<ZO.ZOSimOccurrence>();
            zosimOccurence.DocumentRoot = _documentRoot;


            zosimOccurence.ImportZeroSim(_documentRoot, occurrenceJson);

            // recursively visit all the children of this occurence
            foreach (JObject childOccurrence in occurrenceJson["children"]) {
                OnOccurrence(childOccurrence, occurenceGo, jsonDoc);
            }
        }
        */

        [MenuItem("Zero Sim/Apply Random Colors...")]
        static void ApplyRandomColors() {
            Debug.Log("INFO: Apply Random Colors...");
            GameObject selectedGo = UnityEditor.Selection.activeGameObject;

            if (selectedGo != null) {
                MeshRenderer[] meshRenderers = selectedGo.GetComponentsInChildren<MeshRenderer>();
                foreach (MeshRenderer meshRenderer in meshRenderers) {
                    meshRenderer.sharedMaterial.color = UnityEngine.Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
                }

            }
        }


        public static string MakeRelativePath(string fromPath, string toPath) {
            // if (string.IsNullOrEmpty(fromPath)) throw new ArgumentNullException("fromPath");
            // if (string.IsNullOrEmpty(toPath)) throw new ArgumentNullException("toPath");

            Uri fromUri = new Uri(fromPath);
            Uri toUri = new Uri(toPath);

            if (fromUri.Scheme != toUri.Scheme) { return toPath; } // path can't be made relative.

            Uri relativeUri = fromUri.MakeRelativeUri(toUri);
            String relativePath = Uri.UnescapeDataString(relativeUri.ToString());

            if (toUri.Scheme.Equals("file", StringComparison.InvariantCultureIgnoreCase)) {
                relativePath = relativePath.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
            }

            return relativePath;
        }


    }

}
