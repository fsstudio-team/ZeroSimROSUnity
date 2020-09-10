using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using Newtonsoft.Json.Linq;
using ZO.Util.Extensions;
using ZO.Document;


namespace ZO.Physics {

    /// <summary>
    /// A Zero Sim wrapper class for a Unity Hinge joint. Supports writing and reading to ZeroSim 
    /// JSON format.
    /// 
    /// The HingeJoint groups together 2 rigid bodies, constraining them to move like connected by a hinge.
    ///
    /// The HingeJoint has a motor which can be used to make the hinge spin around the joints axis. A spring 
    /// which attempts to reach for a target angle by spinning around the joints axis. And a limit which 
    /// constrains the joint angle.
    /// </summary>
    [ExecuteAlways]
    public class ZOFixedJoint : MonoBehaviour, ZOSerializationInterface, ZOJointInterface {

        [SerializeField] public UnityEngine.FixedJoint _fixedJoint;

        /// <summary>
        /// The Unity fixed joint linked to this ZOSim fixed joint.
        /// </summary>
        /// <value></value>
        public UnityEngine.FixedJoint UnityFixedJoint {
            get => _fixedJoint;
        }


        #region ZOJointInterface

        /// <summary>
        /// Not applicable for fixed joints.
        /// </summary>
        /// <value></value>
        public float Position {
            get { return 0; }
            set { }
        }

        /// <summary>
        /// Not applicable for fixed joints.
        /// </summary>
        /// <value></value>
        public float Velocity {
            get { return 0; }
            set { }
        }


        /// <summary>
        /// Not applicable for fixed joints.
        /// </summary>
        /// <value></value>
        public float Effort {
            get { return 0; }
            set { }
        }

        /// <summary>
        /// The connected rigid body.  If null then it is the world.
        /// </summary>
        /// <value></value>
        public Rigidbody ConnectedBody {
            get {
                return UnityFixedJoint.connectedBody;
            }
            set {
                UnityFixedJoint.connectedBody = value;
            }
        }

        /// <summary>
        /// The connected ZOSim Occurrence.  Being null does not necessarily mean anything.
        /// </summary>
        /// <value></value>
        public ZOSimOccurrence ConnectedOccurrence {
            get { return ConnectedBody.gameObject.GetComponent<ZOSimOccurrence>(); }
        }




        #endregion



        /// <summary>
        /// Reset is a MonoBehavior method that gets called on creation of this component.
        /// </summary>
        private void Reset() {
            CreateRequirements();
        }

        /// <summary>
        /// Creates the requirements for the ZOFixedJoint including the Unity `HingeJoint`
        /// </summary>
        public void CreateRequirements() {
            // when creating this a ZOFixedJoint we need to create an actual Unity Hinge Joint.
            if (UnityFixedJoint == null) { // create Unity Hinge Joint
                _fixedJoint = gameObject.AddComponent<UnityEngine.FixedJoint>();
            }

            if (_name == null) {
                _name = Type;

                ZOSimOccurrence occurrence = GetComponent<ZOSimOccurrence>();
                if (occurrence) {
                    _name = _name + "_from_" + occurrence.Name;
                }

                if (UnityFixedJoint.connectedBody) {
                    ZOSimOccurrence connected_occurrence = UnityFixedJoint.connectedBody.gameObject.GetComponent<ZOSimOccurrence>();

                    if (connected_occurrence) {
                        _name = _name + "_to_" + connected_occurrence.Name;
                    }
                }
            }

        }


        #region ZOSerializationInterface
        public string Type {
            get { return "joint.fixed"; }
        }

        [SerializeField] public string _name;
        public string Name {
            get {
                return _name;
            }
            private set {
                _name = value;
            }
        }

        private JObject _json;
        public JObject JSON {
            get {
                // if (_json == null) {
                //     _json = BuildJSON();
                // }
                return _json;

            }
        }


        /// <summary>
        /// Serializes the ZOFixedJoint to ZOSim JSON.
        /// </summary>
        /// <param name="documentRoot"></param>
        /// <param name="parent"></param>
        /// <returns></returns>
        public JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            // calculate the world anchor positions relative to the document root transform
            // BUGBUG: maybe from the base of the joint chain which is not necessarily the document root?
            Vector3 worldAnchor = this.transform.TransformPoint(UnityFixedJoint.anchor);
            worldAnchor = documentRoot.transform.InverseTransformPoint(worldAnchor);

            Vector3 worldConnectedAnchor = this.transform.TransformPoint(UnityFixedJoint.connectedAnchor);
            worldConnectedAnchor = documentRoot.transform.InverseTransformPoint(worldConnectedAnchor);


            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("anchor", ZOSimDocumentRoot.ToJSON(UnityFixedJoint.anchor)),
                new JProperty("world_anchor", ZOSimDocumentRoot.ToJSON(worldAnchor)),
                new JProperty("connected_anchor", ZOSimDocumentRoot.ToJSON(UnityFixedJoint.connectedAnchor)),
                new JProperty("world_connected_anchor", ZOSimDocumentRoot.ToJSON(worldConnectedAnchor))
            );

            if (UnityFixedJoint.connectedBody) {
                ZOSimOccurrence connected_occurrence = UnityFixedJoint.connectedBody.gameObject.GetComponent<ZOSimOccurrence>();

                if (connected_occurrence) {
                    json["connected_occurrence"] = connected_occurrence.Name;
                } else {
                    Debug.LogWarning("WARNING: Could not get connected occurrence for ZOFixedJoint: " + Name + "\nPerhaps there is a missing ZOSimOccurrence?");
                }
            } else {
                Debug.LogWarning("WARNING: Could not get connected occurrence for ZOFixedJoint: " + Name);
            }

            ZOSimOccurrence parent_occurrence = GetComponent<ZOSimOccurrence>();
            if (parent_occurrence) {
                json["parent_occurrence"] = parent_occurrence.Name;
            }

            _json = json;

            return json;
        }


        public void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            // Assert.Equals(json["type"].Value<string>() == Type);

            _json = json;
            Name = json.ValueOrDefault("name", Name);
            UnityFixedJoint.anchor = json.ToVector3OrDefault("anchor", UnityFixedJoint.anchor);
            UnityFixedJoint.connectedAnchor = json.ToVector3OrDefault("connected_anchor", UnityFixedJoint.connectedAnchor);



            // find connected body.  this likely will need to be done post LoadFromJSON as it may
            // not be created yet.
            documentRoot.OnPostDeserializationNotification((docRoot) => {
                if (JSON.ContainsKey("connected_occurrence")) {
                    ZOSimOccurrence connectedOccurrence = docRoot.GetOccurrence(JSON["connected_occurrence"].Value<string>());
                    if (connectedOccurrence) {
                        UnityFixedJoint.connectedBody = connectedOccurrence.GetComponent<Rigidbody>();
                    } else {
                        Debug.LogWarning("WARNING: ZOFixedJoint failed to find connected occurrence: " + JSON["connected_occurrence"].Value<string>());
                    }

                }
            });

        }
        #endregion

    }

}
