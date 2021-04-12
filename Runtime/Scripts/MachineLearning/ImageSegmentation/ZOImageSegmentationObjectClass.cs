using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.MachineLearning.ImageSegmentation {
    public class ZOImageSegmentationObjectClass : MonoBehaviour {

        public int _classId = 1;


        // Start is called before the first frame update
        void Start() {

        }

        // Update is called once per frame
        void Update() {

        }

        void OnRenderObject() {
            Renderer[] renderers = GetComponentsInChildren<Renderer>();
            Color color = Color.black;
            string classHexString = _classId.ToString("X4");
            foreach (Renderer renderer in renderers) {
                if (ColorUtility.TryParseHtmlString($"#{classHexString}", out color)) {
                    renderer.material.SetColor("_ClassColor", color);
                }

            }

        }
    }

}
