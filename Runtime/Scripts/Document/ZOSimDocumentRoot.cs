using System.Xml.Linq;
using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using Newtonsoft.Json;
using ZO.ROS.Publisher;
using ZO.ROS.Controllers;
using ZO.ROS.Unity.Service;
using ZO.Math;
using ZO.ImportExport;
namespace ZO.Document {

    /// <summary>
    /// A ZOSim root document. This is the "document root" and "root" GameObject component in the  
    /// Unity object hierarchy. 
    /// 
    /// For example the Unity structure could be:
    /// ```
    /// ZOSimDocumentRoot:
    ///     ZOSimOccurrence:
    ///     ZOSimOccurrence:
    ///         ZOSimOccurrence:
    ///         ZOSimOccurrence:
    ///             ZOSimOccurrence:
    /// ```
    /// 
    /// It is primarily responsible for serialization of the Unity structure to ZoSim JSON document and 
    /// deserialization from ZoSim JSON document to a Unity structure.
    /// </summary>
    [ExecuteAlways]
    public class ZOSimDocumentRoot : MonoBehaviour {


        /// <summary>
        /// The ZOSim JSON document path.
        /// </summary>
        /// <value></value>
        private string _zoSimDocumentFilePath;
        public string ZOSimDocumentFilePath {
            get { return _zoSimDocumentFilePath; }
            set { _zoSimDocumentFilePath = value; }
        }
        




        /// <summary>
        /// The name of this zosim document root. Note that it *should* be unique, so if you are 
        /// spawing a bunch of these make sure to set the "document_name" in the ZeroSim JSON to
        /// something unique!
        /// </summary>
        /// <value></value>
        public string Name {

            get => gameObject.name;
            set => gameObject.name = value;
        }




        /// <summary>
        /// Get zosim occurrence by name.
        /// </summary>
        /// <param name="occurrenceName">the name of the occurrence</param>
        /// <returns>ZOSimOccurrence</returns>
        public ZOSimOccurrence GetOccurrence(string occurrenceName) {
            ZOSimOccurrence[] children = this.transform.GetComponentsInChildren<ZOSimOccurrence>(true);
            foreach(ZOSimOccurrence child in children) {
                if (child.Name == occurrenceName) {
                    return child;
                }
            }
            return null;
        }




        /// <summary>
        /// Convert JSON token to Unity Vector3.
        /// </summary>
        /// <param name="json">JSON Token. Must be a JArray!</param>
        /// <returns></returns>
        static public Vector3 ToVector3(JToken json) {
            List<float> v3list = json.ToObject<List<float>>();
            return new Vector3(v3list[0], v3list[1], v3list[2]);
        }

        /// <summary>
        /// Convert Unity Vector3 to JSON token.
        /// </summary>
        /// <param name="v">Unity Vector3</param>
        /// <returns>JSON token</returns>
        static public JToken ToJSON(Vector3 v) {
            return new JArray(v.x, v.y, v.z);
        }

        /// <summary>
        /// Convert JSON token to Unity Quaternion.
        /// </summary>
        /// <param name="json">JSON Token. Must be a JArray!</param>
        /// <returns>Unity Quaternion</returns>
        static public Quaternion ToQuaternion(JToken json) {
            List<float> qlist = json.ToObject<List<float>>();
            return new Quaternion(qlist[0], qlist[1], qlist[2], qlist[3]);
        }

        /// <summary>
        /// Convert Unity Quaternion to JSON token. 
        /// </summary>
        /// <param name="v">Unity Quaternion</param>
        /// <returns>JSON token</returns>
        static public JToken ToJSON(Quaternion v) {
            return new JArray(v.x, v.y, v.z, v.w);
        }


    }
}