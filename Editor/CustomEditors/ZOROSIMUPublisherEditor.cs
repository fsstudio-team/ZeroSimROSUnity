using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using ZO.ROS.Publisher;

namespace ZO.Editor {

    [CustomEditor(typeof(ZO.ROS.Publisher.ZOROSIMUPublisher))]
    public class ZOROSIMUPublisherEditor : UnityEditor.Editor {

        /// <summary>
        /// Hides unused ROSTopic.  See: https://answers.unity.com/questions/316286/how-to-remove-script-field-in-inspector.html
        /// </summary>
        /// <value></value>
        private static readonly string[] _dontIncludeMe = new string[] { "_updateRateHz" };
        public override void OnInspectorGUI() {

            DrawPropertiesExcluding(serializedObject, _dontIncludeMe);
            serializedObject.ApplyModifiedProperties();

        }
    }

}
