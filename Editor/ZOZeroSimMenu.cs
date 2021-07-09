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


        }

        [MenuItem("GameObject/ZeroSim/Import OBJ...", false, 0)]
        static void ImportOBJ(MenuCommand menuCommand) {
            string filePath = EditorUtility.OpenFilePanel("Import OBJ", ".", "obj");
            string saveToDirectory = EditorUtility.OpenFolderPanel("Save to folder...", Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets"), "");

            if (filePath.Length == 0) {
                return;
            }

            GameObject go = ZOImportOBJ.Import(filePath);

            // build a prefab from the GameObject 
            MeshFilter[] meshFilters = go.GetComponentsInChildren<MeshFilter>();

            foreach (MeshFilter meshFilter in meshFilters) {
                string meshAssetPath = Path.Combine(saveToDirectory, $"{meshFilter.name}.asset");
                meshAssetPath = meshAssetPath.Substring(Application.dataPath.Length);
                Debug.Log("INFO: saving to relative path: " + meshAssetPath);
                Mesh msh = new Mesh();
                msh.vertices = meshFilter.sharedMesh.vertices;
                msh.triangles = meshFilter.sharedMesh.triangles;
                msh.uv = meshFilter.sharedMesh.uv;
                msh.uv2 = meshFilter.sharedMesh.uv2;
                msh.RecalculateNormals();
                msh.RecalculateBounds();

                AssetDatabase.CreateAsset(msh, "Assets/" + meshAssetPath);
            }

        }


    }
}