using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.MachineLearning.ImageSegmentation {
    public class ZOImageSegmentationObjectClass : MonoBehaviour {

        public int _classId = 1;
        public bool _renderClassIdAsVisibleColor = false;

        public int ClassId {
            get => _classId;
            set => _classId = value;
        }



        void OnRenderObject() {
            Renderer[] renderers = GetComponentsInChildren<Renderer>();
            Color color = Color.black;
            foreach (Renderer renderer in renderers) {
                if (_renderClassIdAsVisibleColor) {
                    renderer.material.SetColor("_ClassColor", EncodeIntegerAsColor(ClassId));
                } else {
                    string classHexString = _classId.ToString("X4");
                    if (ColorUtility.TryParseHtmlString($"#{classHexString}", out color)) {
                        renderer.material.SetColor("_ClassColor", color);
                    }

                }
            }
        }

        public static Color EncodeIntegerAsColor(int layer) {
            // Following value must be in the range (0.5 .. 1.0)
            // in order to avoid color overlaps when using 'divider' in this func
            var z = .7f;

            // First 8 layers are Unity Builtin layers
            // Unity supports up to 32 layers in total

            // Lets create palette of unique 16 colors
            var uniqueColors = new Color[] {
                        new Color(1,1,1,1), new Color(z,z,z,1),						// 0
                        new Color(1,1,z,1), new Color(1,z,1,1), new Color(z,1,1,1), // 
                        new Color(1,z,0,1), new Color(z,0,1,1), new Color(0,1,z,1), // 7
                        
                        new Color(1,0,0,1), new Color(0,1,0,1), new Color(0,0,1,1), // 8
                        new Color(1,1,0,1), new Color(1,0,1,1), new Color(0,1,1,1), // 
                        new Color(1,z,z,1), new Color(z,1,z,1)						// 15
		    };

            // Create as many colors as necessary by using base 16 color palette
            // To create more than 16 - will simply adjust brightness with 'divider'
            var color = uniqueColors[layer % uniqueColors.Length];
            var divider = 1.0f + Mathf.Floor(layer / uniqueColors.Length);
            color /= divider;

            return color;
        }

    }

}
