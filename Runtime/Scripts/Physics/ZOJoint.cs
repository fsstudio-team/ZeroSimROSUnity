using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;

namespace ZO.Physics {

    [ExecuteAlways]
    public class ZOJoint : MonoBehaviour, ZO.ZOSerializationInterface {

        [SerializeField] public UnityEngine.Joint _unityJoint;
        public UnityEngine.Joint UnityJoint {
            get {
                return _unityJoint;
            }
        }

        [SerializeField] public string _name;

        public string Type {
            get {
                if (UnityJoint.GetType() == typeof(FixedJoint)) {
                    return "fixed";
                } else if (UnityJoint.GetType() == typeof(HingeJoint)) {
                    return "revolute";
                }

                return "none";
            }
        }

        public string Name {
            get {
                return _name;
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

        void Start() {

            if (Application.IsPlaying(gameObject) == false) { // In Editor Mode 

                if (UnityJoint == null) { // create Unity Hinge Joint
                    _unityJoint = gameObject.AddComponent<UnityEngine.HingeJoint>();
                }

                if (_name == null) {
                    _name = Type;

                    ZOSimOccurrence occurrence = GetComponent<ZOSimOccurrence>();
                    if (occurrence) {
                        _name = _name + "_from_" + occurrence.Name;
                    }

                    if (UnityJoint.connectedBody) {
                        ZOSimOccurrence connected_occurrence = UnityJoint.connectedBody.gameObject.GetComponent<ZOSimOccurrence>();

                        if (connected_occurrence) {
                            _name = _name + "_to_" + connected_occurrence.Name;
                        }

                    }
                }
            }
        }

        // private void OnDestroy() {
        //     if ((Application.isEditor ==true) && (Application.isPlaying == false) && (_hingeJoint != null) && (Application.isLoadingLevel == false)) {
        //         DestroyImmediate(_hingeJoint);
        //     }
        // }

        public JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("anchor", ZOSimDocumentRoot.ToJSON(UnityJoint.anchor)),
                new JProperty("axis", ZOSimDocumentRoot.ToJSON(UnityJoint.axis)),
                new JProperty("connected_anchor", ZOSimDocumentRoot.ToJSON(UnityJoint.connectedAnchor))

            // new JProperty("use_spring", UnityJoint.useSpring),
            // new JProperty("spring", new JObject(
            //     new JProperty("spring", UnityJoint.spring.spring),
            //     new JProperty("damper", UnityJoint.spring.damper),
            //     new JProperty("target_position", UnityJoint.spring.targetPosition)
            // )),
            // new JProperty("use_motor", UnityJoint.useMotor),
            // new JProperty("motor", new JObject(
            //     new JProperty("target_velocity", UnityJoint.motor.targetVelocity),
            //     new JProperty("force", UnityJoint.motor.force),
            //     new JProperty("free_spin", UnityJoint.motor.freeSpin)
            // )),
            // new JProperty("use_limits", UnityJoint.useLimits),
            // new JProperty("limits", new JObject(
            //     new JProperty("min", UnityJoint.limits.min),
            //     new JProperty("max", UnityJoint.limits.max),
            //     new JProperty("bounciness", UnityJoint.limits.bounciness),
            //     new JProperty("bounce_min_velocity", UnityJoint.limits.bounceMinVelocity),
            //     new JProperty("contact_distance", UnityJoint.limits.contactDistance)
            // ))
            );

            if (UnityJoint.connectedBody) {
                ZOSimOccurrence connected_occurrence = UnityJoint.connectedBody.gameObject.GetComponent<ZOSimOccurrence>();

                if (connected_occurrence) {
                    json["connected_occurrence"] = connected_occurrence.Name;
                }
            }

            ZOSimOccurrence parent_occurrence = GetComponent<ZOSimOccurrence>();
            if (parent_occurrence) {
                json["parent_occurrence"] = parent_occurrence.Name;
            }
            _json = json;

            return json;
        }

        public void ImportZeroSim(ZOSimDocumentRoot documentRoot, JObject json) {
            throw new System.NotImplementedException("TODO!");
            // TODO:
        }


        public void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            throw new System.NotImplementedException("TODO!");
        }

    }

}
