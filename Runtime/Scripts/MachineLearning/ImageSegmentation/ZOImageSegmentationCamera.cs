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
    }

}
