using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using ZO.ROS.Publisher;

namespace ZO.Editor {

    [CustomEditor(typeof(ZO.ROS.Publisher.ZOROSTransformPublisher))]
    public class ZOROSTransformPublisherEditor : UnityEditor.Editor {

        // int _hingeChoiceIndex = 0;
        string[] _hingeJointChoices;
        public override void OnInspectorGUI() {

            DrawDefaultInspector();

            
            
        }
    }

}
