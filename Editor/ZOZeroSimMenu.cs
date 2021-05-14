using UnityEngine;
using UnityEditor;
using ZO.Document;

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
    }
}