using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using Newtonsoft.Json;

namespace ZO {

    [ExecuteAlways]
    /// <summary>
    /// A ZOSim root docuemtn. This is the "document root" and first component in a ZoSim definition.
    /// </summary>
    public class ZOSimDocumentRoot : MonoBehaviour {


        /// <summary>
        /// The on di
        /// </summary>
        /// <value></value>
        [SerializeField] public string _zoSimDocumentFilePath;
        public string ZOSimDocumentFilePath {
            get { return _zoSimDocumentFilePath; }
            set { _zoSimDocumentFilePath = value; }
        }
        private JObject _json;
        public JObject JSON {
            get => _json;
            set => _json = value;
        }
        private List<JObject> _components = null;
        
        private List<Action<ZOSimDocumentRoot>> _postLoadFromJSONNotifiers = new List<Action<ZOSimDocumentRoot>>();

        /// <summary>
        /// If a ZOSimTypeInterface needs to be notified after a LoadFrom a ZOSim File.
        /// For example a ZOHingeJoint needs to fixup the connected bodies that may or
        /// may not exist during the LoadFromJSON call.
        /// </summary>
        /// <param name="notification"></param>
        public void AddPostLoadFromJSONNotification(Action<ZOSimDocumentRoot> notification) {
            _postLoadFromJSONNotifiers.Add(notification);
        } 


        // Start is called before the first frame update
        void Start() {
            _json = BuildZOSimDocument();
        }

        public JObject GetComponentJSON(string componentName) {
            // find the component associated with the with the occurrence
            if (_components == null) {
                _components = _json["components"].ToObject<List<JObject>>();
            }

            JObject componentJson = _components.Find(item => string.Equals(item["name"].Value<string>(), componentName));
            return componentJson;

        }

        public JObject GetOccurrenceJSON(string occurrenceName, JArray occurrences = null) {
            if (occurrences == null) {
                occurrences = (JArray)JSON["occurrences"];
            }

            foreach (JObject oc in occurrences) {
                if (oc["name"].Value<string>() == occurrenceName) {
                    return oc;
                }

                // search children
                JArray children = (JArray)oc["children"];
                if (children != null) {
                    JObject childrenResult = GetOccurrenceJSON(occurrenceName, children);
                    if (childrenResult != null) {
                        return childrenResult;
                    }

                }

            }

            return null;

        }

        public ZOSimOccurrence GetOccurrence(string occurrenceName) {
            Transform t = transform.Find(occurrenceName);
            if (t != null) {
                ZOSimOccurrence simOccurrence = t.GetComponent<ZOSimOccurrence>();
                return simOccurrence;
            }
            return null;
        }

        /// <summary>
        /// Builds ZOSim document from traversing hierarchy.
        /// </summary>
        /// <returns></returns>
        public JObject BuildZOSimDocument() {
            // create new ZeroSim JSON document from scratch
            JObject zoSimDocumentJSON = new JObject(
                new JProperty("document_name", gameObject.name),
                new JProperty("document_version", "1.0"),
                new JProperty("mesh_scale", new JArray(new float[] { 1.0f, 1.0f, 1.0f })),
                new JProperty("transform_scale", new JArray(new float[] { 1.0f, 1.0f, 1.0f })),
                new JProperty("components", new JArray()),
                new JProperty("occurrences", new JArray())
            );

            // we are always the "root" component
            zoSimDocumentJSON["components"].Value<JArray>().Add(new JObject(
                new JProperty("name", gameObject.name)
            ));

            // build the occurrences
            JArray occurrences = new JArray();
            foreach (Transform child in transform) {
                ZOSimOccurrence zoSimOccurrence = child.GetComponent<ZOSimOccurrence>();
                if (zoSimOccurrence) {
                    JObject occurrenceJSON = zoSimOccurrence.BuildJSON(this);
                    zoSimOccurrence.JSON = occurrenceJSON;

                    occurrences.Add(occurrenceJSON);
                }
            }

            zoSimDocumentJSON["occurrences"] = occurrences;


            return zoSimDocumentJSON;
        }

        /// <summary>
        /// Saves to ZOSim file.
        /// </summary>
        public void SaveToZOSimFile(string filePath) {
            JSON = BuildZOSimDocument();
            // Save ZoSim file
            using (StreamWriter streamWriter = File.CreateText(filePath)) {
                streamWriter.Write(JSON.ToString());
            }
        }

        /// <summary>
        /// Load Zero Sim JSON file.
        /// </summary>
        /// <param name="filePath">path to zosim file</param>
        public void LoadFromZOSimFile(string filePath) {

            if (File.Exists(filePath) == true) { // load from file
                using (StreamReader file = File.OpenText(filePath)) {
                    using (JsonTextReader reader = new JsonTextReader(file)) {
                        JSON = (JObject)JToken.ReadFrom(reader);
                    }
                }

                // load json file
                LoadFromJSON(JSON);

            } else {
                Debug.LogError("ERROR: Could not load ZoSim Project. File does not exist: " + filePath);
            }
        }

        public void LoadFromJSON(JObject json) {
            JSON = json;

            gameObject.name = JSON["document_name"].Value<string>();

            // go through occurrences                
            foreach (JObject occurreneJSON in JSON["occurrences"].Value<JArray>()) {
                string occurrenceName = occurreneJSON["name"].Value<string>();

                ZOSimOccurrence simOccurrence = GetOccurrence(occurrenceName);
                if (simOccurrence == null) {
                    // create new sim occurrence gameobject
                    GameObject go = new GameObject(occurrenceName);
                    go.transform.parent = this.transform;
                    simOccurrence = go.AddComponent<ZOSimOccurrence>();
                }
                simOccurrence.LoadFromJSON(this, occurreneJSON);
            }

            // notify anyone who needs to do any fixup post load
            foreach(Action<ZOSimDocumentRoot> postLoadNotify in _postLoadFromJSONNotifiers) {
                postLoadNotify(this);
            }
            _postLoadFromJSONNotifiers.Clear();
            // TODO: store components
            // TODO: remove any occurrences that are not in the zosim file
            // TODO: check if visual mesh files exists
            // TODO: check if collision mesh files exists


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