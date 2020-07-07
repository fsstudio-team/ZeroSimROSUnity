using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using Newtonsoft.Json.Linq;
using ZO.Util.Extensions;

namespace ZO.Physics {

    /// <summary>
    /// A Zero Sim wrapper class for a Unity Hinge joint. Supports writing and reading to ZeroSim 
    /// JSON format.
    /// </summary>
    [ExecuteAlways]
    public class ZOHingeJoint : MonoBehaviour, ZO.ZOSimTypeInterface {

        [SerializeField] public UnityEngine.HingeJoint _hingeJoint;
        public UnityEngine.HingeJoint UnityHingeJoint {
            get {
                return _hingeJoint;
            }
        }

        [SerializeField] public string _name;

        public string Type {
            get { return "joint.hinge"; }
        }

        public string Name {
            get {
                return _name;
            }
            private set {
                _name = value;
            }
        }

        //TODO: move the actual motor update into FixedUpdate otherwise it won't work
        public float AngularVelocityDegrees {
            set {
                JointMotor motor = _hingeJoint.motor;
                motor.targetVelocity = value;
                motor.freeSpin = false;
                _hingeJoint.motor = motor;
                _hingeJoint.useMotor = true;
                _hingeJoint.useSpring = false;
            }

            get {
                return UnityHingeJoint.velocity;
            }
        }

        private JObject _json;
        public JObject JSON {
            get {
                if (_json == null) {
                    _json = BuildJSON();
                }
                return _json;

            }
        }

        void Start() {

            if (Application.IsPlaying(gameObject) == false) { // In Editor Mode 

            }
        }

        private void Reset() {
            CreateRequirements();    
        }

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



        public JObject BuildJSON(UnityEngine.Object parent = null) {
            JObject hingedJointJSON = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("anchor", ZOSimDocumentRoot.ToJSON(UnityHingeJoint.anchor)),
                new JProperty("axis", ZOSimDocumentRoot.ToJSON(UnityHingeJoint.axis)),
                new JProperty("connected_anchor", ZOSimDocumentRoot.ToJSON(UnityHingeJoint.connectedAnchor)),
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
                    hingedJointJSON["connected_occurrence"] = connected_occurrence.Name;
                }
            }

            ZOSimOccurrence parent_occurrence = GetComponent<ZOSimOccurrence>();
            if (parent_occurrence) {
                hingedJointJSON["parent_occurrence"] = parent_occurrence.Name;
            }


            return hingedJointJSON;
        }

        public void ImportZeroSim(JObject json) {
            throw new System.NotImplementedException("TODO!");
            // TODO:
        }

        public void LoadFromJSON(JObject json) {
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

            // TODO find connected body.  this likely will need to be done post LoadFromJSON as it may
            // not be created yet.



        }


    }

}
