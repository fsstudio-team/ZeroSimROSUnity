using System.Linq;
using System;
using System.Xml.Linq;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEditor;
using Newtonsoft.Json.Linq;
using ZO.Physics;
using ZO.Util.Extensions;
using ZO.ROS.Controllers;
using ZO.Sensors;
using ZO.ROS.Unity;
using ZO.ROS.Publisher;
using ZO.Math;
using ZO.ImportExport;

namespace ZO.Document {

    /// <summary>
    /// A `ZOSimOccurence` is similar to a Unity GameObject but contains additional "meta" info specific
    /// to ZoSim.  It is responsible for serialization/deserialization of ZoSim JSON.  
    /// Every Unity GameObject that needs to serialize or interact with Zero Sim should
    /// have a `ZOSimOccurrence` as a component. 
    /// </summary>
    public class ZOSimOccurrence : MonoBehaviour {

        [ZO.Util.ZOReadOnly] [SerializeField] public ZOSimDocumentRoot _documentRoot;


        /// <summary>
        /// Every ZoSim configuration has a `ZOSimDocumentRoot` as the root component.
        /// This property returns the ZoSim document root of this configuration.
        /// </summary>
        /// <value></value>
        public ZOSimDocumentRoot DocumentRoot {
            set { _documentRoot = value; }
            get {
                if (_documentRoot == null) {  // traverse up hierarchy to find parent base component
                    Transform parent = transform.parent;
                    while (parent != null) {
                        _documentRoot = parent.GetComponent<ZOSimDocumentRoot>();
                        if (_documentRoot != null) {
                            break;  // found
                        }
                        parent = parent.transform.parent; // keep traversing up
                    }
                }

                Assert.IsNotNull(_documentRoot, "ERROR: a ZOSimOccurrence needs a ZOSimDocumentRoot at the root of the hierarchy.");

                return _documentRoot;
            }
        }

        private void OnValidate() {
            // update root component
            ZOSimDocumentRoot rootComponent = DocumentRoot;
        }

        private void Start() {
            if (Application.IsPlaying(gameObject) == false) { // In Editor Mode 
                // update root component
                ZOSimDocumentRoot rootComponent = DocumentRoot;
            }
        }

        /// <summary>
        /// Finds an occurrence in this configuragtion 
        /// </summary>
        /// <param name="occurrenceName"></param>
        /// <returns></returns>
        public ZOSimOccurrence GetOccurrence(string occurrenceName) {
            Transform t = transform.Find(occurrenceName);
            if (t != null) {
                ZOSimOccurrence simOccurrence = t.GetComponent<ZOSimOccurrence>();
                return simOccurrence;
            }
            return null;
        }


        /// <summary>
        /// Finds a hinge joint by name.
        /// </summary>
        /// <param name="name"></param>
        /// <returns></returns>
        public ZOHingeJoint GetHingeJointNamed(string name) {
            ZOHingeJoint[] hingeJoints = GetComponents<ZOHingeJoint>();
            foreach (ZOHingeJoint hingeJoint in hingeJoints) {
                if (hingeJoint.Name == name) {
                    return hingeJoint;
                }
            }
            return null;
        }


        public string Name {
            get {
                return gameObject.name;
            }
            private set => gameObject.name = value;
        }

        public string Type {
            get {
                return "occurrence";
            }
        }


    }
}