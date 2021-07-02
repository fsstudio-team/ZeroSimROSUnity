using UnityEngine;
using UnityEditor;
using System.Linq;
using System.IO;

namespace ZO.Import {
    public class ZOImportZeroSimProcessor : AssetPostprocessor {

        public static bool IsProcessingZOSimColliders {
            get; set;
        }
        /// <summary>
        /// This handles when draging and dropping a zosim project file
        /// </summary>
        // private void OnPreprocessAsset() {
        //     ZOImportZeroSim.ZeroSimProjectFile = this.assetPath;
        //     ZOImportZeroSim.ImportFusion360Project();

        // }


        // See: https://gist.github.com/ACap99/61e309688a53377b0dceaa7efce1a8e3 & https://stackoverflow.com/questions/53931129/cant-create-collision-mesh-for-a-child-gameobject
        void OnPostprocessModel(GameObject go) {

            // Add convex mesh collider and disable rendering for collision meshes
            if (IsProcessingZOSimColliders) {

                
                Debug.Log("INFO: ZOImportZeroSimProcessor::OnPostprocessModel: " + go.name);
                

                // BUGBUG: always assuming the 0 child game object 
                GameObject meshGameObject = go.transform.GetChild(0).gameObject;

                // create the meshcollider
                UnityEngine.MeshCollider meshCollider = meshGameObject.AddComponent<UnityEngine.MeshCollider>();
                meshCollider.convex = true;

                // disable meshrenderer for collider
                meshGameObject.GetComponent<MeshRenderer>().enabled = false;

                // save back prefab
                string assetPath = AssetDatabase.GetAssetPath(go);
                if (assetPath.Length > 0) {
                    PrefabUtility.SaveAsPrefabAsset(go, assetPath);
                } else {
                    Debug.LogWarning("WARNING: invalid asset path for: " + go.name);
                }


            }
        }

    }
}
