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
    public class ZOHingeJoint : ZOGameObjectBase, ZOJointInterface {

        public Rigidbody _connectedBody;
        public Vector3 _anchor = Vector3.zero;
        public Vector3 _axis = Vector3.forward;

        [Header("Joint Motor Control")]
        public bool _useMotor = true;
        public float _motorForce = 1.0f;

        [Header("Joint Torsional Spring Control")]
        public bool _useSpring = false;
        public float _springConstant = 100.0f;
        public float _springDampening = 20.0f;

        [SerializeField] [ZOReadOnlyAttribute] public UnityEngine.HingeJoint _hingeJoint;

        /// <summary>
        /// The Unity hinge joint linked to this ZOSim hinge joint.
        /// </summary>
        /// <value></value>
        public UnityEngine.HingeJoint UnityHingeJoint {
            get {
                return _hingeJoint;
            }
        }

        /// <summary>
        /// Flag to indicate if using joint motor or freewheeling.
        /// </summary>
        /// <value></value>
        public bool UseMotor {
            get {
                return UnityHingeJoint.useMotor;
            }
            set {
                _useMotor = value;
                UnityHingeJoint.useMotor = value;
            }
        }

        /// <summary>
        /// The maximum force the motor uses.
        /// </summary>
        /// <value></value>
        public float MotorForce {
            get {
                return UnityHingeJoint.motor.force;
            }
            set {
                JointMotor jointMotor = UnityHingeJoint.motor;
                jointMotor.force = value;
                UnityHingeJoint.motor = jointMotor;
            }
        }


        /// <summary>
        /// Use spring to drive joint.
        /// </summary>
        /// <value></value>
        public bool UseSpring {
            get {
                return UnityHingeJoint.useSpring;
            }
            set {
                _useSpring = value;
                UnityHingeJoint.useSpring = value;
            }
        }

        /// <summary>
        /// Spring constant force.
        /// </summary>
        /// <value></value>
        public float SpringConstant {
            get {
                return UnityHingeJoint.spring.spring;
            }
            set {
                JointSpring spring = UnityHingeJoint.spring;
                _springConstant = value;
                spring.spring = value;
                UnityHingeJoint.spring = spring;
            }
        }

        /// <summary>
        /// The damper force used to dampen the spring.
        /// </summary>
        /// <value></value>
        public float SpringDampening {
            get {
                return UnityHingeJoint.spring.damper;
            }
            set {
                JointSpring spring = UnityHingeJoint.spring;
                _springDampening = value;
                spring.damper = value;
                UnityHingeJoint.spring = spring;
            }
        }

        /// <summary>
        /// The direction of axis in which the body is constrained.
        /// </summary>
        /// <value></value>
        public Vector3 Axis {
            get {
                return UnityHingeJoint.axis;
            }
            set {
                _axis = value;
                UnityHingeJoint.axis = value;
            }
        }

        /// <summary>
        /// The position of the anchor in which the joint body is constrained
        /// </summary>
        /// <value></value>
        public Vector3 Anchor {
            get {
                return UnityHingeJoint.anchor;
            }
            set {
                _anchor = value;
                UnityHingeJoint.anchor = value;
            }
        }

        public Vector3 ConnectedAnchor {
            get {
                return UnityHingeJoint.connectedAnchor;
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
                if (UnityHingeJoint.connectedBody != value) {
                    _connectedBody = value;
                    UnityHingeJoint.connectedBody = value;
                }

                // update the name
                if (string.IsNullOrEmpty(Name)) {
                    Name = Type;

                    ZOSimOccurrence occurrence = GetComponent<ZOSimOccurrence>();
                    if (occurrence) {
                        Name = Name + "_from_" + occurrence.Name;
                    }

                    if (UnityHingeJoint.connectedBody) {
                        ZOSimOccurrence connected_occurrence = UnityHingeJoint.connectedBody.gameObject.GetComponent<ZOSimOccurrence>();

                        if (connected_occurrence) {
                            Name = Name + "_to_" + connected_occurrence.Name;
                        }
                    }

                }

            }
        }


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
            get {
                return UnityHingeJoint.velocity * Mathf.Deg2Rad;
            }

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
        /// The effort that is applied to the hinge (Nm)
        /// </summary>
        /// <value></value>
        public float Effort {
            get { return UnityHingeJoint.motor.force; }
            set {
                // TODO
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
        protected override void ZOStart() {
            base.ZOStart();
            // calculate starting "zero" joint angle in radians
            // NOTE: this has to be done because there is a bug in the Unity HingeJoint angle :-/
            if (ConnectedBody) {
                Quaternion r = Quaternion.Inverse(this.transform.rotation) * ConnectedBody.transform.rotation;
                _startAngleRadians = ZO.Math.ZOMathUtil.FindQuaternionTwist(r, UnityHingeJoint.axis);
            }
        }

        protected override void ZOFixedUpdate() {
            base.ZOFixedUpdate();
            // calculate joint angle relative to the starting joint
            // NOTE: this has to be done because there is a bug in the Unity HingeJoint angle :-/
            Quaternion r = Quaternion.Inverse(this.transform.rotation) * ConnectedBody.transform.rotation;
            float angle = ZO.Math.ZOMathUtil.FindQuaternionTwist(r, UnityHingeJoint.axis);
            _currentAngleRadians = angle - _startAngleRadians;

        }

        protected override void ZOOnGUI() {
            base.ZOOnGUI();
            if (_debug) {
                // Vector3 worldAxis = this.transform.rotation * UnityHingeJoint.axis;
                // ZOSimOccurrence occurrence = GetComponent<ZOSimOccurrence>();
                // ZOSimDocumentRoot documentRoot = occurrence.DocumentRoot;
                // worldAxis = documentRoot.transform.InverseTransformDirection(worldAxis);

                // float angle = Vector3.SignedAngle(transform.forward, ConnectedBody.transform.forward, worldAxis);
                // Quaternion r = Quaternion.Inverse(this.transform.rotation) * ConnectedBody.transform.rotation;
                // float angle = ZO.Math.ZOMathUtil.FindQuaternionTwist(r, UnityHingeJoint.axis) * Mathf.Rad2Deg;

                GUI.TextField(new Rect(10, 10, 400, 22), this.Name + " Angle: " + (Position * Mathf.Rad2Deg).ToString("R2")
                                + " Target: " + (UnityHingeJoint.spring.targetPosition * Mathf.Rad2Deg).ToString("R2"));

            }
        }

        protected override void ZOOnValidate() {
            base.ZOOnValidate();
            Axis = _axis;
            ConnectedBody = _connectedBody;
            Anchor = _anchor;
            UseMotor = _useMotor;
            MotorForce = _motorForce;
            UseSpring = _useSpring;
            SpringConstant = _springConstant;
            SpringDampening = _springDampening;
        }

        #endregion

        // private void FixedUpdate() {
        //     Debug.Log("Joint: " + Name + " Angle: " + UnityHingeJoint.angle.ToString("n2"));
        // }

        /// <summary>
        /// Reset is a MonoBehavior method that gets called on creation of this component.
        /// </summary>
        protected override void ZOReset() {
            base.ZOReset();
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

            if (string.IsNullOrEmpty(Name)) {
                Name = Type;

                ZOSimOccurrence occurrence = GetComponent<ZOSimOccurrence>();
                if (occurrence) {
                    Name = Name + "_from_" + occurrence.Name;
                }

                if (UnityHingeJoint.connectedBody) {
                    ZOSimOccurrence connected_occurrence = UnityHingeJoint.connectedBody.gameObject.GetComponent<ZOSimOccurrence>();

                    if (connected_occurrence) {
                        Name = Name + "_to_" + connected_occurrence.Name;
                    }
                }
            }

        }

        // private void OnDestroy() {
        //     if ((Application.isEditor ==true) && (Application.isPlaying == false) && (_hingeJoint != null) && (Application.isLoadingLevel == false)) {
        //         DestroyImmediate(_hingeJoint);
        //     }
        // }


        public string Type {
            get { return "joint.hinge"; }
        }

        [SerializeField] public string _name;
        public string Name {
            get {
                return _name;
            }
            set {
                _name = value;
            }
        }

    }

}
