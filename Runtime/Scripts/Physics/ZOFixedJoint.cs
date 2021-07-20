using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using Newtonsoft.Json.Linq;
using ZO.Util.Extensions;
using ZO.Document;
using ZO.Util;


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
    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(HingeJoint))]
    public class ZOFixedJoint : ZOGameObjectBase, ZOJointInterface {

        [SerializeField] public UnityEngine.FixedJoint _fixedJoint;
        public Rigidbody _connectedBody;

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
        /// <summary>
        /// The connected rigid body.  If null then it is the world.
        /// </summary>
        /// <value></value>
        public Rigidbody ConnectedBody {
            get {
                return UnityFixedJoint.connectedBody;
            }
            set {
                if (UnityFixedJoint.connectedBody != value) {
                    _connectedBody = value;
                    UnityFixedJoint.connectedBody = value;
                }

                // update the name
                if (string.IsNullOrEmpty(Name)) {
                    Name = Type;

                    ZOSimOccurrence occurrence = GetComponent<ZOSimOccurrence>();
                    if (occurrence) {
                        Name = Name + "_from_" + occurrence.Name;
                    }

                    if (UnityFixedJoint.connectedBody) {
                        ZOSimOccurrence connected_occurrence = UnityFixedJoint.connectedBody.gameObject.GetComponent<ZOSimOccurrence>();

                        if (connected_occurrence) {
                            Name = Name + "_to_" + connected_occurrence.Name;
                        }
                    }

                }

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


        protected override void ZOOnValidate() {
            base.ZOOnValidate();
            ConnectedBody = _connectedBody;
        }


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


    }

}
