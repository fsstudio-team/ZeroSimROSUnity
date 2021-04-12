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
            Renderer renderer = GetComponent<Renderer>();
            // renderer.material.SetOverrideTag("RenderType", "MyClass");
            if (_classId == 1) {
                renderer.material.SetColor("_ClassColor", Color.green);
            } else if (_classId == 2) {
                renderer.material.SetColor("_ClassColor", Color.yellow);
            }
            
        }
    }

}
