using UnityEngine;
using UnityEditor;
using ZO.Document;
namespace ZO.Editor {

    [CustomEditor(typeof(ZOSimOccurrence))]
    public class ZOSimOccurrenceEditor : UnityEditor.Editor {

        public override void OnInspectorGUI() {

            DrawDefaultInspector();

            ZOSimOccurrence simOccurrence = (ZOSimOccurrence)target;

            if (GUILayout.Button("Create Child Occurrence")) {
                GameObject baseOccurrence = new GameObject($"ChildOf{simOccurrence.Name}");
                baseOccurrence.transform.SetParent(simOccurrence.transform, false);
                ZOSimOccurrence occurrence = baseOccurrence.AddComponent<ZOSimOccurrence>();
                GameObjectUtility.EnsureUniqueNameForSibling(baseOccurrence);



                GameObject visuals = new GameObject("visuals");
                visuals.transform.SetParent(occurrence.transform, false);

                GameObject collisions = new GameObject("collisions");
                collisions.transform.SetParent(occurrence.transform, false);

            }

            if (GUILayout.Button("Create Sibling Occurrence")) {
                GameObject baseOccurrence = new GameObject($"SiblingOf{simOccurrence.Name}");
                baseOccurrence.transform.SetParent(simOccurrence.transform.parent, false);
                ZOSimOccurrence occurrence = baseOccurrence.AddComponent<ZOSimOccurrence>();
                GameObjectUtility.EnsureUniqueNameForSibling(baseOccurrence);


                GameObject visuals = new GameObject("visuals");
                visuals.transform.SetParent(occurrence.transform, false);

                GameObject collisions = new GameObject("collisions");
                collisions.transform.SetParent(occurrence.transform, false);

            }

        }
    }
}