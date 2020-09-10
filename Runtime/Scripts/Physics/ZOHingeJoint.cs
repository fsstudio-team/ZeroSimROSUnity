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
    public class ZOHingeJoint : MonoBehaviour, ZOSerializationInterface, ZOJointInterface {

        [SerializeField] public UnityEngine.HingeJoint _hingeJoint;

        /// <summary>
        /// The Unity hinge joint linked to this ZOSim hinge joint.
        /// </summary>
        /// <value></value>
        public UnityEngine.HingeJoint UnityHingeJoint {
            get {
                return _hingeJoint;
            }
        }

        public bool _debug = false;


        #region ZOJointInterface

        /// <summary>
        /// The hinge angle in radians.  
        /// </summary>
        /// <value></value>
        public float Position {
            get {
                // _connectedBodyStartRotation.
                // NOTE:  There is longstanding bug in Unity Hinge Joint angle where 
                // return UnityHingeJoint.angle * Mathf.Deg2Rad;
                return _currentAngleRadians;
            }
            set {
                JointSpring spring = UnityHingeJoint.spring;
                UnityHingeJoint.spring = spring;
                spring.targetPosition = value * Mathf.Rad2Deg;
                UnityHingeJoint.spring = spring;
                UnityHingeJoint.useSpring = true;
                UnityHingeJoint.useMotor = false;
            }
        }

        /// <summary>
        /// The velocity of the hinge in rad/s
        /// </summary>
        /// <value></value>
        public float Velocity {
            get { return UnityHingeJoint.velocity * Mathf.Deg2Rad; }

            set {
                JointMotor motor = UnityHingeJoint.motor;
                motor.targetVelocity = value * Mathf.Rad2Deg;
                motor.freeSpin = false;
                UnityHingeJoint.motor = motor;
                UnityHingeJoint.useMotor = true;
                UnityHingeJoint.useSpring = false;
            }

        }


        /// <summary>
        /// The effor that is applied to the hinge Nm
        /// </summary>
        /// <value></value>
        public float Effort {
            get { return UnityHingeJoint.motor.force; }
            set {
                // TODO
            }
        }

        /// <summary>
        /// The connected rigid body.  If null then it is the world.
        /// </summary>
        /// <value></value>
        public Rigidbody ConnectedBody {
            get {
                return UnityHingeJoint.connectedBody;
            }
            set {
                UnityHingeJoint.connectedBody = value;
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
        /// Read only maximum torque of this hinge joint motor.
        /// </summary>
        /// <value></value>
        // public float TorqueNewtonMeters {
        //     get {
        //         return UnityHingeJoint.motor.force;
        //     }
        // }


        private float _startAngleRadians = 0;
        private float _currentAngleRadians = 0;

        #region MonoBehaviour
        private void Start() {
            // calculate starting "zero" joint angle in radians
            // NOTE: this has to be done because there is a bug in the Unity HingeJoint angle :-/
            Quaternion r = Quaternion.Inverse(this.transform.rotation) * ConnectedBody.transform.rotation;
            _startAngleRadians = ZO.Math.ZOMathUtil.FindQuaternionTwist(r, UnityHingeJoint.axis);

        }

        private void FixedUpdate() {
            // calculate joint angle relative to the starting joint
            // NOTE: this has to be done because there is a bug in the Unity HingeJoint angle :-/
            Quaternion r = Quaternion.Inverse(this.transform.rotation) * ConnectedBody.transform.rotation;
            float angle = ZO.Math.ZOMathUtil.FindQuaternionTwist(r, UnityHingeJoint.axis);
            _currentAngleRadians = angle - _startAngleRadians;

        }

        private void OnGUI() {
            if (_debug) {
                // Vector3 worldAxis = this.transform.rotation * UnityHingeJoint.axis;
                // ZOSimOccurrence occurrence = GetComponent<ZOSimOccurrence>();
                // ZOSimDocumentRoot documentRoot = occurrence.DocumentRoot;
                // worldAxis = documentRoot.transform.InverseTransformDirection(worldAxis);

                // float angle = Vector3.SignedAngle(transform.forward, ConnectedBody.transform.forward, worldAxis);
                // Quaternion r = Quaternion.Inverse(this.transform.rotation) * ConnectedBody.transform.rotation;
                // float angle = ZO.Math.ZOMathUtil.FindQuaternionTwist(r, UnityHingeJoint.axis) * Mathf.Rad2Deg;

                GUI.TextField(new Rect(10, 10, 300, 22), this.Name + " Angle: " + (_currentAngleRadians * Mathf.Rad2Deg).ToString("R2") 
                                + " Target: " + (UnityHingeJoint.spring.targetPosition * Mathf.Rad2Deg).ToString("R2"));

            }
        }

        #endregion

        // private void FixedUpdate() {
        //     Debug.Log("Joint: " + Name + " Angle: " + UnityHingeJoint.angle.ToString("n2"));
        // }

        /// <summary>
        /// Reset is a MonoBehavior method that gets called on creation of this component.
        /// </summary>
        private void Reset() {
            CreateRequirements();
        }

        /// <summary>
        /// Creates the requirements for the ZOHingeJoint including the Unity `HingeJoint`
        /// </summary>
        public void CreateRequirements() {
            // when creating this a ZOHingeJoint we need to create an actual Unity Hinge Joint.
            if (UnityHingeJoint == null) { // create Unity Hinge Joint
                _hingeJoint = gameObject.AddComponent<UnityEngine.HingeJoint>();
            }

            if (_name == null) {
                _name = Type;

                ZOSimOccurrence occurrence = GetComponent<ZOSimOccurrence>();
                if (occurrence) {
                    _name = _name + "_from_" + occurrence.Name;
                }

                if (UnityHingeJoint.connectedBody) {
                    ZOSimOccurrence connected_occurrence = UnityHingeJoint.connectedBody.gameObject.GetComponent<ZOSimOccurrence>();

                    if (connected_occurrence) {
                        _name = _name + "_to_" + connected_occurrence.Name;
                    }
                }
            }

        }

        // private void OnDestroy() {
        //     if ((Application.isEditor ==true) && (Application.isPlaying == false) && (_hingeJoint != null) && (Application.isLoadingLevel == false)) {
        //         DestroyImmediate(_hingeJoint);
        //     }
        // }


        #region ZOSerializationInterface
        public string Type {
            get { return "joint.hinge"; }
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
        /// Serializes the ZOHingeJoint to ZOSim JSON.
        /// </summary>
        /// <param name="documentRoot"></param>
        /// <param name="parent"></param>
        /// <returns></returns>
        public JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            // calculate the world anchor positions relative to the document root transform
            // BUGBUG: maybe from the base of the joint chain which is not necessarily the document root?
            Vector3 worldAnchor = this.transform.TransformPoint(UnityHingeJoint.anchor);
            worldAnchor = documentRoot.transform.InverseTransformPoint(worldAnchor);

            Vector3 worldConnectedAnchor = this.transform.TransformPoint(UnityHingeJoint.connectedAnchor);
            worldConnectedAnchor = documentRoot.transform.InverseTransformPoint(worldConnectedAnchor);

            Vector3 worldAxis = this.transform.rotation * UnityHingeJoint.axis;
            worldAxis = documentRoot.transform.InverseTransformDirection(worldAxis);

            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("anchor", ZOSimDocumentRoot.ToJSON(UnityHingeJoint.anchor)),
                new JProperty("world_anchor", ZOSimDocumentRoot.ToJSON(worldAnchor)),
                new JProperty("axis", ZOSimDocumentRoot.ToJSON(UnityHingeJoint.axis)),
                new JProperty("world_axis", ZOSimDocumentRoot.ToJSON(worldAxis)),
                new JProperty("connected_anchor", ZOSimDocumentRoot.ToJSON(UnityHingeJoint.connectedAnchor)),
                new JProperty("world_connected_anchor", ZOSimDocumentRoot.ToJSON(worldConnectedAnchor)),
                new JProperty("use_spring", UnityHingeJoint.useSpring),
                new JProperty("spring", new JObject(
                    new JProperty("spring", UnityHingeJoint.spring.spring),
                    new JProperty("damper", UnityHingeJoint.spring.damper),
                    new JProperty("target_position", UnityHingeJoint.spring.targetPosition)
                )),
                new JProperty("use_motor", UnityHingeJoint.useMotor),
                new JProperty("motor", new JObject(
                    new JProperty("target_velocity", UnityHingeJoint.motor.targetVelocity),
                    new JProperty("force", UnityHingeJoint.motor.force),
                    new JProperty("free_spin", UnityHingeJoint.motor.freeSpin)
                )),
                new JProperty("use_limits", UnityHingeJoint.useLimits),
                new JProperty("limits", new JObject(
                    new JProperty("min", UnityHingeJoint.limits.min),
                    new JProperty("max", UnityHingeJoint.limits.max),
                    new JProperty("bounciness", UnityHingeJoint.limits.bounciness),
                    new JProperty("bounce_min_velocity", UnityHingeJoint.limits.bounceMinVelocity),
                    new JProperty("contact_distance", UnityHingeJoint.limits.contactDistance)
                ))
            );

            if (UnityHingeJoint.connectedBody) {
                ZOSimOccurrence connected_occurrence = UnityHingeJoint.connectedBody.gameObject.GetComponent<ZOSimOccurrence>();

                if (connected_occurrence) {
                    json["connected_occurrence"] = connected_occurrence.Name;
                } else {
                    Debug.LogWarning("WARNING: Could not get connected occurrence for ZOHingeJoint: " + Name + "\nPerhaps there is a missing ZOSimOccurrence?");
                }
            } else {
                Debug.LogWarning("WARNING: Could not get connected occurrence for ZOHingeJoint: " + Name);
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
            UnityHingeJoint.anchor = json.ToVector3OrDefault("anchor", UnityHingeJoint.anchor);
            UnityHingeJoint.axis = json.ToVector3OrDefault("axis", UnityHingeJoint.axis);
            UnityHingeJoint.connectedAnchor = json.ToVector3OrDefault("connected_anchor", UnityHingeJoint.connectedAnchor);
            UnityHingeJoint.useSpring = json.ValueOrDefault<bool>("use_spring", UnityHingeJoint.useSpring);


            if (json.ContainsKey("spring")) {
                JObject springJSON = json["spring"].Value<JObject>();
                JointSpring spring = UnityHingeJoint.spring;
                spring.spring = springJSON.ValueOrDefault<float>("spring", spring.spring);
                spring.damper = springJSON.ValueOrDefault<float>("damper", spring.damper);
                spring.targetPosition = springJSON.ValueOrDefault<float>("target_position", spring.targetPosition);
                UnityHingeJoint.spring = spring;

            }

            UnityHingeJoint.useMotor = json.ValueOrDefault<bool>("use_motor", UnityHingeJoint.useMotor);
            if (json.ContainsKey("use_motor")) {
                JObject motorJSON = json["motor"].Value<JObject>();
                JointMotor motor = UnityHingeJoint.motor;
                motor.targetVelocity = motorJSON.ValueOrDefault<float>("target_velocity", motor.targetVelocity);
                motor.force = motorJSON.ValueOrDefault<float>("force", motor.force);
                motor.freeSpin = motorJSON.ValueOrDefault<bool>("free_spin", motor.freeSpin);
                UnityHingeJoint.motor = motor;
            }

            UnityHingeJoint.useLimits = json.ValueOrDefault<bool>("use_limits", UnityHingeJoint.useLimits);
            if (json.ContainsKey("limits")) {
                JObject limitsJSON = json["limits"].Value<JObject>();
                JointLimits limits = UnityHingeJoint.limits;
                limits.min = limitsJSON.ValueOrDefault<float>("min", limits.min);
                limits.max = limitsJSON.ValueOrDefault<float>("max", limits.max);
                limits.bounciness = limitsJSON.ValueOrDefault<float>("bounciness", limits.bounciness);
                limits.bounceMinVelocity = limitsJSON.ValueOrDefault<float>("bounce_min_velocity", UnityHingeJoint.limits.bounceMinVelocity);
                limits.contactDistance = limitsJSON.ValueOrDefault<float>("contact_distance", limits.contactDistance);
                UnityHingeJoint.limits = limits;
            }

            // find connected body.  this likely will need to be done post LoadFromJSON as it may
            // not be created yet.
            documentRoot.OnPostDeserializationNotification((docRoot) => {
                if (JSON.ContainsKey("connected_occurrence")) {
                    ZOSimOccurrence connectedOccurrence = docRoot.GetOccurrence(JSON["connected_occurrence"].Value<string>());
                    if (connectedOccurrence) {
                        UnityHingeJoint.connectedBody = connectedOccurrence.GetComponent<Rigidbody>();
                    } else {
                        Debug.LogWarning("WARNING: ZOHingeJoint failed to find connected occurrence: " + JSON["connected_occurrence"].Value<string>());
                    }

                }
            });

        }
        #endregion

    }

}
