using UnityEngine;
using UnityEditor;
using ZO.Document;
using ZO.ImportExport;
using System.IO;
namespace ZO.Editor {
    public static class ZOZeroSimMenu {
        [MenuItem("GameObject/ZeroSim/New Robot", false, 0)]
        static void CreateZeroSimRobot(MenuCommand menuCommand) {
            GameObject documentRoot = new GameObject("MyRobot");
            ZOSimDocumentRoot docRoot = documentRoot.AddComponent<ZOSimDocumentRoot>();

            GameObject baseOccurrence = new GameObject("base");
            baseOccurrence.transform.SetParent(documentRoot.transform);
            ZOSimOccurrence occurrence = baseOccurrence.AddComponent<ZOSimOccurrence>();
            occurrence.DocumentRoot = docRoot;


            GameObject visuals = new GameObject("visuals");
            visuals.transform.SetParent(occurrence.transform);

            GameObject collisions = new GameObject("collisions");
            collisions.transform.SetParent(occurrence.transform);

            // create default cube visual
            GameObject cubeVisual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cubeVisual.name = "MyExampleVisualCube";
            cubeVisual.transform.SetParent(visuals.transform);
            BoxCollider boxCollider = cubeVisual.GetComponent<BoxCollider>();
            GameObject.DestroyImmediate(boxCollider);

            // create default collision cube
            GameObject cubeCollision = new GameObject("MyExampleCollisionCube");
            cubeCollision.AddComponent<BoxCollider>();
            cubeCollision.transform.SetParent(collisions.transform);



            Undo.RegisterCreatedObjectUndo(documentRoot, "Create ZeroSim Robot " + documentRoot.name);
            Selection.activeGameObject = documentRoot;
        }



        [MenuItem("GameObject/ZeroSim/Import URDF...", false, 0)]
        static void ImportURDF(MenuCommand menuCommand) {
            string filePath = EditorUtility.OpenFilePanel("Import URDF", ".", "urdf");
            string saveToDirectory = EditorUtility.OpenFolderPanel("Save to folder...", Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets"), "");

            if (filePath.Length == 0) {
                return;
            }

            ZOSimDocumentRoot documentRoot = ZOImportURDF.Import(filePath);
            GameObject prefabGo = MakeGameObjectPrefab(documentRoot.gameObject, saveToDirectory);
            Object.DestroyImmediate(documentRoot.gameObject);
            PrefabUtility.InstantiatePrefab(prefabGo);

        }

        [MenuItem("GameObject/ZeroSim/Import OBJ...", false, 0)]
        static void ImportOBJ(MenuCommand menuCommand) {
            string filePath = EditorUtility.OpenFilePanel("Import OBJ", ".", "obj");
            string saveToDirectory = EditorUtility.OpenFolderPanel("Save to folder...", Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets"), "");

            if (string.IsNullOrEmpty(filePath) == true) {
                return;
            }

            GameObject go = ZOImportOBJ.Import(filePath);
            GameObject prefabGo = MakeGameObjectPrefab(go, saveToDirectory);
            Object.DestroyImmediate(go);
            PrefabUtility.InstantiatePrefab(prefabGo);


        }

        static GameObject MakeGameObjectPrefab(GameObject gameObject, string saveToDirectory) {

            // save any meshes and materials 
            MeshFilter[] meshFilters = gameObject.GetComponentsInChildren<MeshFilter>();
            MeshRenderer[] meshRenderers = gameObject.GetComponentsInChildren<MeshRenderer>();

            foreach (MeshRenderer meshRenderer in meshRenderers) {
                SaveMaterial(meshRenderer.sharedMaterial, saveToDirectory);
            }


            foreach (MeshFilter meshFilter in meshFilters) {
                SaveMesh(meshFilter.sharedMesh, saveToDirectory);
            }

            string path = Path.Combine(saveToDirectory, gameObject.name + ".prefab");
            path = FileUtil.GetProjectRelativePath(path);

            return PrefabUtility.SaveAsPrefabAsset(gameObject, path);

        }


        public static void SaveMaterial(Material material, string saveToDirectory) {
            string path = Path.Combine(saveToDirectory, material.name + ".mat");
            path = FileUtil.GetProjectRelativePath(path);
            path = AssetDatabase.GenerateUniqueAssetPath(path);
            Debug.Log($"INFO: Saving material at path: {path}");
            AssetDatabase.CreateAsset(material, path);
            AssetDatabase.SaveAssets();
        }

        public static void SaveMesh(Mesh mesh, string saveToDirectory) {
            string path = Path.Combine(saveToDirectory, mesh.name + ".asset");
            path = FileUtil.GetProjectRelativePath(path);
            Debug.Log($"INFO: Saving mesh at path: {path}");
            path = AssetDatabase.GenerateUniqueAssetPath(path);
            AssetDatabase.CreateAsset(mesh, path);
            AssetDatabase.SaveAssets();

        }


    }
}