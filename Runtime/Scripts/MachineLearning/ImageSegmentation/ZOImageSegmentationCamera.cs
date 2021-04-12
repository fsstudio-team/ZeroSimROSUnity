using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ZO.Util;
using ZO.Sensors;


namespace ZO.MachineLearning.ImageSegmentation {

    /// <summary>
    /// Camera to generate image segmentation data for ML algorithms like semantic segmentation.
    /// </summary>
    [RequireComponent(typeof(Camera))] //[ExecuteInEditMode]
    public class ZOImageSegmentationCamera : ZORGBCamera {

        [Header("Image Segmentation Parameters")]
        public Shader _replacementShader;

        protected override void ZOOnValidate() {
            base.ZOOnValidate();

            
#if UNITY_EDITOR
            // get the default replacement shader
            _replacementShader = Shader.Find("ZeroSim/ImageSegmentationShader");

            // set camera clear flags and background to 0
            UnityCamera.clearFlags = CameraClearFlags.Color;
            UnityCamera.backgroundColor = Color.black;

#endif // UNITY_EDITOR
        }

        protected override void ZOStart() {
            base.ZOStart();

            UnityCamera = GetComponent<Camera>();
            UnityCamera.SetReplacementShader(_replacementShader, "RenderType");
            Shader.EnableKeyword("RED_COL");

            ZOImageSegmentationObjectClass[] classTags = GameObject.FindObjectsOfType<ZOImageSegmentationObjectClass>();

            foreach (ZOImageSegmentationObjectClass go in classTags) {
                Renderer renderer = go.GetComponent<Renderer>();
                renderer.material.SetOverrideTag("RenderType", "MyClass");
            }

        }
    }

}
