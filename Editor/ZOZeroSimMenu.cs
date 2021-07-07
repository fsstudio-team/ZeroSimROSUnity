using UnityEngine;
using UnityEditor;
using ZO.Document;
using ZO.ImportExport;
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

            if (filePath.Length == 0) {
                return;
            }

            ZOImportURDF.Import(filePath);

        }

        [MenuItem("GameObject/ZeroSim/Import OBJ...", false, 0)]
        static void ImportOBJ(MenuCommand menuCommand) {
            string filePath = EditorUtility.OpenFilePanel("Import OBJ", ".", "obj");

            if (filePath.Length == 0) {
                return;
            }

            ZOImportOBJ.Import(filePath);

        }


    }
}