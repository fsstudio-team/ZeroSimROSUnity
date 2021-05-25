using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEngine.Assertions;
using Newtonsoft.Json.Linq;
using ZO.Util.Extensions;
using ZO.Document;
using ZO.Util;


namespace ZO.Physics {

    /// <summary>
    /// A Zero Sim wrapper class for a Prismatic Joint (aka Slider Joint)
    /// 
    /// The PrismaticJoint groups together 2 rigid bodies, constraining them to move like connected linear track.
    ///
    /// </summary>
    [ExecuteAlways]
    public class ZOPrismaticJoint : MonoBehaviour, ZOJointInterface {


        [Serializable]
        public struct PrismaticJointLimits {
            public float LowerLimit;
            public float UpperLimit;
            public float Bounciness;
        }

        [Serializable]
        public struct PrismaticJointSpring {
            public float SpringForce;
            public float DampingForce;
            public float Position;
        }

        [Serializable]
        public struct PrismaticJointMotor {
            public float Velocity;
            public float MaxForce;
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

        public string Type {
            get { return "joint.prismatic"; }
        }


        public Rigidbody _connectedBody;
        public Vector3 _anchor = Vector3.zero;
        public Vector3 _axis = Vector3.forward;
        public bool _useSpring = false;
        public JointSpring _spring = new JointSpring();
        public bool _useMotor = false;
        public JointMotor _motor = new JointMotor();
        public bool _useLimits = false;
        public PrismaticJointLimits _jointLimits = new PrismaticJointLimits();
        public PrismaticJointLimits JointLimits {
            get { return _jointLimits; }
            set {

                _jointLimits = value;

                _jointLimits.LowerLimit = Mathf.Min(_jointLimits.LowerLimit, 0);
                _jointLimits.UpperLimit = Mathf.Max(_jointLimits.UpperLimit, 0);
                _jointLimits.Bounciness = Mathf.Clamp01(_jointLimits.Bounciness);


                _prismaticLimits.bounciness = JointLimits.Bounciness;
                if (_useLimits == true) {
                    _prismaticLimits.limit = (JointLimits.UpperLimit - JointLimits.LowerLimit) * 0.5f;
                    UnityConfigurableJoint.xMotion = ConfigurableJointMotion.Limited;

                } else {
                    UnityConfigurableJoint.xMotion = ConfigurableJointMotion.Free;
                }

                UnityConfigurableJoint.linearLimit = _prismaticLimits;

                if (ConnectedBody != null) {

                    Vector3 lowerLimit = transform.TransformPoint(Anchor + (JointDirection * JointLimits.LowerLimit));
                    Vector3 upperLimit = transform.TransformPoint(Anchor + (JointDirection * JointLimits.UpperLimit));
                    Vector3 midPoint = (lowerLimit + upperLimit) * 0.5f;
                    Vector3 localMidpoint = ConnectedBody.transform.InverseTransformPoint(midPoint);

                    UnityConfigurableJoint.connectedAnchor = localMidpoint;
                } else {
                    //TODO:
                }


            }
        }

        public bool _enableCollisions = true;

        [SerializeField] [ZOReadOnlyAttribute] protected ConfigurableJoint _configurableJoint;

        private SoftJointLimit _prismaticLimits = new SoftJointLimit();
        protected SoftJointLimit PrismaticLimits {
            get { return _prismaticLimits; }
        }

        public bool UseLimits {
            get { return _useLimits; }
            set {
                // set up the limits
                _useLimits = value;
                JointLimits = _jointLimits;
            }

        }

        public bool UseSpring {
            get { return _useSpring; }
        }

        public bool UseMotor {
            get { return _useMotor; }
        }

        public bool EnableCollisions {
            get { return UnityConfigurableJoint.enableCollision; }
            set {
                _enableCollisions = value;
                UnityConfigurableJoint.enableCollision = value;
            }
        }

        public Vector3 JointDirection {
            get {
                if (ConnectedBody != null) {
                    return UnityConfigurableJoint.connectedBody.transform.InverseTransformDirection(transform.TransformDirection(Axis.normalized));
                } else {
                    return transform.TransformDirection(Axis);
                }
            }
        }


        public JointMotor UnityJointMotor {
            get { return _motor; }
            set { _motor = value; }
        }

        protected JointDrive _jointDrive;

        public JointDrive UnityJointDrive {
            get { return _jointDrive; }
            set { _jointDrive = value; }
        }

        protected SoftJointLimit _softJointLimit;
        public SoftJointLimit UnitySoftJointLimit {
            get { return _softJointLimit; }
            set { _softJointLimit = value; }
        }

        /// <summary>
        /// The Unity hinge joint linked to this ZOSim hinge joint.
        /// </summary>
        /// <value></value>
        public ConfigurableJoint UnityConfigurableJoint {
            get {
                return _configurableJoint;
            }
        }

        /// <summary>
        /// The direction of axis in which the body is constrained.
        /// </summary>
        /// <value></value>
        public Vector3 Axis {
            get {
                return -UnityConfigurableJoint.axis;
            }
            set {
                UnityConfigurableJoint.axis = -value;
            }
        }

        /// <summary>
        /// The position of the anchor in which the joint body is constrained
        /// </summary>
        /// <value></value>
        public Vector3 Anchor {
            get {
                return UnityConfigurableJoint.anchor;
            }
            set {
                UnityConfigurableJoint.anchor = value;

                if (ConnectedBody != null) {

                    Vector3 lowerLimit = transform.TransformPoint(Anchor + (JointDirection * JointLimits.LowerLimit));
                    Vector3 upperLimit = transform.TransformPoint(Anchor + (JointDirection * JointLimits.UpperLimit));
                    Vector3 midPoint = (lowerLimit + upperLimit) * 0.5f;
                    Vector3 localMidpoint = ConnectedBody.transform.InverseTransformPoint(midPoint);

                    UnityConfigurableJoint.connectedAnchor = localMidpoint;
                } else {
                    //TODO

                }

            }
        }


        public Vector3 ConnectedAnchor {
            get {
                return UnityConfigurableJoint.connectedAnchor;
            }
            set {
                // Vector3 newConnectedAnchor = value;
                if (ConnectedBody != null) {

                    Vector3 lowerLimit = transform.TransformPoint(Anchor + (JointDirection * JointLimits.LowerLimit));
                    Vector3 upperLimit = transform.TransformPoint(Anchor + (JointDirection * JointLimits.UpperLimit));
                    Vector3 midPoint = (lowerLimit + upperLimit) * 0.5f;
                    Vector3 localMidpoint = ConnectedBody.transform.InverseTransformPoint(midPoint);

                    UnityConfigurableJoint.connectedAnchor = localMidpoint;
                } else {
                    //TODO:
                }
            }
        }

        public bool _debug = false;


        #region ZOJointInterface

        /// <summary>
        /// The local position along joint
        /// </summary>
        /// <value></value>
        public float Position {
            get;
            set;
        }

        /// <summary>
        /// The velocity 
        /// </summary>
        /// <value></value>
        public float Velocity {
            get; set;
        }


        /// <summary>
        /// The effort that is applied to the slider
        /// </summary>
        /// <value></value>
        public float Effort {
            get { return UnityJointMotor.force; }
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
                return UnityConfigurableJoint.connectedBody;
            }
            set {
                if (UnityConfigurableJoint.connectedBody != value) {
                    UnityConfigurableJoint.connectedBody = value;
                    if (UnityConfigurableJoint.connectedBody != null) {
                        ConnectedAnchor = UnityConfigurableJoint.connectedBody.transform.InverseTransformPoint(Anchor);
                    } else {
                        ConnectedAnchor = transform.TransformPoint(Anchor);
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

        #region MonoBehaviour


#if UNITY_EDITOR
        protected void OnDrawGizmosSelected() {
            if (_debug) {


                Quaternion worldAxisRotation = Quaternion.LookRotation(transform.TransformDirection(Axis));

                Vector3 lowerLimit = transform.TransformPoint(Anchor + (JointDirection * JointLimits.LowerLimit));
                Handles.color = Color.red;
                Handles.CylinderHandleCap(0, lowerLimit, worldAxisRotation, HandleUtility.GetHandleSize(lowerLimit) * 0.2f, EventType.Repaint);

                Vector3 upperLimit = transform.TransformPoint(Anchor + (JointDirection * JointLimits.UpperLimit));
                Handles.color = Color.blue;
                Handles.CylinderHandleCap(0, upperLimit, worldAxisRotation, HandleUtility.GetHandleSize(lowerLimit) * 0.2f, EventType.Repaint);

                Handles.color = Color.white;
                Handles.DrawDottedLine(lowerLimit, upperLimit, 5.0f);

                Vector3 anchor = transform.TransformPoint(Anchor);
                Handles.color = Color.cyan;
                Handles.SphereHandleCap(0, anchor, worldAxisRotation, HandleUtility.GetHandleSize(lowerLimit) * 0.2f, EventType.Repaint);

                if (ConnectedBody) {
                    Vector3 connectedAnchor = ConnectedBody.transform.TransformPoint(ConnectedAnchor);
                    Handles.color = Color.magenta;
                    Handles.SphereHandleCap(0, connectedAnchor, worldAxisRotation, HandleUtility.GetHandleSize(lowerLimit) * 0.2f, EventType.Repaint);
                }
            }

        }
#endif // UNITY_EDITOR

        /// <summary>
        /// Reset is a MonoBehavior method that gets called on creation of this component.
        /// </summary>
        private void Reset() {
            CreateRequirements();

        }

        private void OnValidate() {
            SetupPrismaticJointFromConfigurableJoint();
        }



        #endregion // MonoBehaviour

        /// <summary>
        /// Creates the requirements for the ZOHingeJoint including the Unity `HingeJoint`
        /// </summary>
        protected void CreateRequirements() {
            // when creating this a ZOHingeJoint we need to create an actual Unity Hinge Joint.
            if (UnityConfigurableJoint == null) { // create Unity Hinge Joint
                _configurableJoint = gameObject.AddComponent<ConfigurableJoint>();
                SetupPrismaticJointFromConfigurableJoint();
            }

            if (_name == null) {
                _name = Type;

                ZOSimOccurrence occurrence = GetComponent<ZOSimOccurrence>();
                if (occurrence) {
                    _name = _name + "_from_" + occurrence.Name;
                }

                if (UnityConfigurableJoint.connectedBody) {
                    ZOSimOccurrence connected_occurrence = UnityConfigurableJoint.connectedBody.gameObject.GetComponent<ZOSimOccurrence>();

                    if (connected_occurrence) {
                        _name = _name + "_to_" + connected_occurrence.Name;
                    }
                }
            }
        }

        protected void SetupPrismaticJointFromConfigurableJoint() {
            UnityConfigurableJoint.autoConfigureConnectedAnchor = false;
            UnityConfigurableJoint.yMotion = ConfigurableJointMotion.Locked;
            UnityConfigurableJoint.zMotion = ConfigurableJointMotion.Locked;
            UnityConfigurableJoint.angularXMotion = ConfigurableJointMotion.Locked;
            UnityConfigurableJoint.angularYMotion = ConfigurableJointMotion.Locked;
            UnityConfigurableJoint.angularZMotion = ConfigurableJointMotion.Locked;
            Axis = _axis;
            ConnectedBody = _connectedBody;
            Anchor = _anchor;
            UseLimits = _useLimits;
            JointLimits = _jointLimits;
            EnableCollisions = _enableCollisions;

        }

        // private void OnDestroy() {
        //     if ((Application.isEditor ==true) && (Application.isPlaying == false) && (_hingeJoint != null) && (Application.isLoadingLevel == false)) {
        //         DestroyImmediate(_hingeJoint);
        //     }
        // }



    }

}
