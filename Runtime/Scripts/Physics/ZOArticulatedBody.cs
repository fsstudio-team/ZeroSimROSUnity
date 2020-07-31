using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using Newtonsoft.Json.Linq;
using ZO.Util.Extensions;
using ZO;

namespace ZO.Physics {

    /// <summary>
    /// A Zero Sim wrapper class for a Unity Articulated Body Supports writing and reading to ZeroSim 
    /// JSON format.
    /// 
    /// A body that forms part of a Physics articulation.
    /// An articulation is a set of bodies arranged in a logical tree. The parent-child link in this tree 
    /// reflects that the bodies have their relative motion constrained. Articulations are solved by a 
    /// Featherstone solver that works in reduced coordinates - that is each body has relative coordinates 
    /// to its parent but only along the unlocked degrees of freedom. This guarantees there is no unwanted 
    /// stretch.
    /// 
    /// Like with regular Joints, there are two anchors for each pair of connected articulation bodies. One 
    /// anchor is defined in the parent body's reference frame, whereas the other one is defined in the 
    /// child's reference frame. Changing the constraints, you directly affect the allowed space for relative 
    /// positions of the two anchors. For instance, ArticulationDofLock.LockedMotion will not allow any 
    /// relative motion at all.
    /// </summary>
    public class ZOArticulatedBody : MonoBehaviour, ZOSerializationInterface, ZOJointInterface {

        public ArticulationBody _articulationBody;

        /// <summary>
        /// Access to the Unity articulation body.
        /// </summary>
        /// <value></value>
        public ArticulationBody UnityArticulationBody {
            get {
                return _articulationBody;
            }
            private set => _articulationBody = value;
        }


        /// <summary>
        /// Gets the joint type string.
        /// </summary>
        /// <value></value>
        public string ArticulationBodyJointType {
            get {
                return UnityArticulationBody.jointType.ToString().ToLower();
            }
        }


        /// <summary>
        /// Reset is a Unity call that gets called on creation in the editor.
        /// </summary>
        private void Reset() {
            CreateRequirements();
        }

        /// <summary>
        /// Creates requirements for the articulated body, including the actual Unity `ArticulationBody`
        /// and the name of the joint.
        /// </summary>
        public void CreateRequirements() {

            // create if it doesn't exist
            if (UnityArticulationBody == null) { // create Unity Hinge Joint
                UnityArticulationBody = gameObject.AddComponent<ArticulationBody>();
            }

            // build the name from occurrences if they exist
            if (_name == null) {
                _name = Type;

                ZOSimOccurrence occurrence = GetComponent<ZOSimOccurrence>();
                if (occurrence) {
                    _name = _name + "_from_" + occurrence.Name;
                }

                ZOSimOccurrence connected_occurrence = null;
                if (UnityArticulationBody.transform.childCount > 0) {
                    // BUGBUG: this first child may not actually be the connected child?
                    connected_occurrence = UnityArticulationBody.transform.GetChild(0).gameObject.GetComponent<ZOSimOccurrence>();
                }

                if (connected_occurrence) {
                    _name = _name + "_to_" + connected_occurrence.Name;
                }
            }

        }

        /// <summary>
        /// The joint position.  
        /// 
        /// For a hinge/revolute it would be the angle in radians.
        /// 
        /// For a linear/prismatic it would be the distance from center in meters.
        /// </summary>
        /// <value></value>
        public float Position {
            get {
                if (UnityArticulationBody.jointType == ArticulationJointType.FixedJoint) {
                    return 0; // fixed joints don't have a joint position
                }
                return UnityArticulationBody.jointPosition[0];
            }

            set {
                ArticulationDrive drive = UnityArticulationBody.xDrive;
                drive.target = value * Mathf.Rad2Deg; // remember to convert from radians to degrees
                UnityArticulationBody.xDrive = drive;
            }
        }

        /// <summary>
        /// The velocity of the joint (rad/s or m/s)
        /// </summary>
        /// <value></value>
        public float Velocity {
            get {
                if (UnityArticulationBody.jointType == ArticulationJointType.FixedJoint) {
                    return 0; // fixed joints don't have a joint position
                }

                return UnityArticulationBody.jointVelocity[0];
            }

            set {
                // TODO
            }
        }


        /// <summary>
        /// The effort that is applied to the joint (Nm or N)
        /// </summary>
        /// <value></value>
        public float Effort {
            get {
                if (UnityArticulationBody.jointType == ArticulationJointType.FixedJoint) {
                    return 0; // fixed joints don't have a joint position
                }

                return UnityArticulationBody.jointForce[0];
            }

            set {
                // TODO
            }
        }





        #region ZOSerializationInterface
        public string Type {
            get { return "joint.articulated_body." + ArticulationBodyJointType; }
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
                return _json;
            }
        }


        public JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            // calculate the world anchor positions relative to the document root transform
            // BUGBUG: maybe from the base of the joint chain which is not necessarily the document root?
            Vector3 worldAnchor = this.transform.TransformPoint(UnityArticulationBody.anchorPosition);
            worldAnchor = documentRoot.transform.InverseTransformPoint(worldAnchor);
            Vector3 worldAxis = this.transform.rotation * (UnityArticulationBody.anchorRotation * Vector3.right);  // BUGBUG: always assuming "right" axis?
            worldAxis = documentRoot.transform.InverseTransformDirection(worldAxis);

            // get the parent occurrence name if it exists
            string parentOccurrenceName = "";
            if (this.transform.parent && this.transform.parent.GetComponent<ZOSimOccurrence>()) {
                parentOccurrenceName = this.transform.parent.GetComponent<ZOSimOccurrence>().Name;
            }

            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("owner_occurrence", this.GetComponent<ZOSimOccurrence>().Name),
                new JProperty("parent_occurrence", parentOccurrenceName),
                new JProperty("mass", UnityArticulationBody.mass),
                new JProperty("use_gravity", UnityArticulationBody.useGravity),
                new JProperty("world_axis", ZOSimDocumentRoot.ToJSON(worldAxis)),
                new JProperty("world_anchor", ZOSimDocumentRoot.ToJSON(worldAnchor)),
                new JProperty("anchor_position", ZOSimDocumentRoot.ToJSON(UnityArticulationBody.anchorPosition)),
                new JProperty("anchor_rotation_quaternion", ZOSimDocumentRoot.ToJSON(UnityArticulationBody.anchorRotation)),

                new JProperty("parent_anchor_position", ZOSimDocumentRoot.ToJSON(UnityArticulationBody.parentAnchorPosition)),
                new JProperty("parent_anchor_rotation_quaternion", ZOSimDocumentRoot.ToJSON(UnityArticulationBody.parentAnchorRotation)),

                new JProperty("linear_damping", UnityArticulationBody.linearDamping),
                new JProperty("angular_damping", UnityArticulationBody.angularDamping),
                new JProperty("joint_friction", UnityArticulationBody.jointFriction),

                new JProperty("is_immovable", UnityArticulationBody.immovable),
                new JProperty("is_root", UnityArticulationBody.isRoot),

                new JProperty("x_drive", new JObject(
                    new JProperty("damping", UnityArticulationBody.xDrive.damping),
                    new JProperty("force_limit", UnityArticulationBody.xDrive.forceLimit),
                    new JProperty("lower_limit", UnityArticulationBody.xDrive.lowerLimit),
                    new JProperty("upper_limit", UnityArticulationBody.xDrive.upperLimit),
                    new JProperty("stiffness", UnityArticulationBody.xDrive.stiffness),
                    new JProperty("target", UnityArticulationBody.xDrive.target),
                    new JProperty("target_velocity", UnityArticulationBody.xDrive.targetVelocity)
                )),
                new JProperty("y_drive", new JObject(
                    new JProperty("damping", UnityArticulationBody.yDrive.damping),
                    new JProperty("force_limit", UnityArticulationBody.yDrive.forceLimit),
                    new JProperty("lower_limit", UnityArticulationBody.yDrive.lowerLimit),
                    new JProperty("upper_limit", UnityArticulationBody.yDrive.upperLimit),
                    new JProperty("stiffness", UnityArticulationBody.yDrive.stiffness),
                    new JProperty("target", UnityArticulationBody.yDrive.target),
                    new JProperty("target_velocity", UnityArticulationBody.yDrive.targetVelocity)
                )),
                new JProperty("z_drive", new JObject(
                    new JProperty("damping", UnityArticulationBody.zDrive.damping),
                    new JProperty("force_limit", UnityArticulationBody.zDrive.forceLimit),
                    new JProperty("lower_limit", UnityArticulationBody.zDrive.lowerLimit),
                    new JProperty("upper_limit", UnityArticulationBody.zDrive.upperLimit),
                    new JProperty("stiffness", UnityArticulationBody.zDrive.stiffness),
                    new JProperty("target", UnityArticulationBody.zDrive.target),
                    new JProperty("target_velocity", UnityArticulationBody.zDrive.targetVelocity)
                ))
            );

            _json = json;

            return json;
        }


        public void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            // Assert.Equals(json["type"].Value<string>() == Type);

            _json = json;
            Name = json.ValueOrDefault("name", Name);

            string type = json["type"].Value<string>();
            if (type == "joint.articulated_body.fixedjoint") {
                UnityArticulationBody.jointType = ArticulationJointType.FixedJoint;
            } else if (type == "joint.articulated_body.revolutejoint") {
                UnityArticulationBody.jointType = ArticulationJointType.RevoluteJoint;
            } else {
                // TODO: implement other joint types
                throw new NotImplementedException("ERROR: Joint type not yet supported!  TODO!!!");
            }

            UnityArticulationBody.mass = json.ValueOrDefault("mass", UnityArticulationBody.mass);
            UnityArticulationBody.useGravity = json.ValueOrDefault("use_gravity", UnityArticulationBody.useGravity);
            // UnityArticulationBody.immovable = json.ValueOrDefault("is_immovable", UnityArticulationBody.immovable);

            UnityArticulationBody.parentAnchorPosition = json.ToVector3OrDefault("anchor_position", UnityArticulationBody.parentAnchorPosition);
            UnityArticulationBody.parentAnchorRotation = json.ToQuaternionOrDefault("anchor_rotation_quaternion", UnityArticulationBody.parentAnchorRotation);
            UnityArticulationBody.linearDamping = json.ValueOrDefault("linear_damping", UnityArticulationBody.linearDamping);
            UnityArticulationBody.angularDamping = json.ValueOrDefault("angular_damping", UnityArticulationBody.linearDamping);
            UnityArticulationBody.jointFriction = json.ValueOrDefault("joint_friction", UnityArticulationBody.jointFriction);

            if (json.ContainsKey("x_drive")) {
                JObject driveJSON = json["x_drive"].Value<JObject>();
                ArticulationDrive drive = UnityArticulationBody.xDrive;
                drive.damping = driveJSON.ValueOrDefault("damping", drive.damping);
                drive.forceLimit = driveJSON.ValueOrDefault("force_limit", drive.forceLimit);
                drive.lowerLimit = driveJSON.ValueOrDefault("lower_limit", drive.lowerLimit);
                drive.upperLimit = driveJSON.ValueOrDefault("upper_limit", drive.upperLimit);
                drive.stiffness = driveJSON.ValueOrDefault("stiffness", drive.stiffness);
                drive.target = driveJSON.ValueOrDefault("target", drive.target);
                drive.targetVelocity = driveJSON.ValueOrDefault("target_velocity", drive.targetVelocity);
                UnityArticulationBody.xDrive = drive;
            }

            if (json.ContainsKey("y_drive")) {
                JObject driveJSON = json["y_drive"].Value<JObject>();
                ArticulationDrive drive = UnityArticulationBody.xDrive;
                drive.damping = driveJSON.ValueOrDefault("damping", drive.damping);
                drive.forceLimit = driveJSON.ValueOrDefault("force_limit", drive.forceLimit);
                drive.lowerLimit = driveJSON.ValueOrDefault("lower_limit", drive.lowerLimit);
                drive.upperLimit = driveJSON.ValueOrDefault("upper_limit", drive.upperLimit);
                drive.stiffness = driveJSON.ValueOrDefault("stiffness", drive.stiffness);
                drive.target = driveJSON.ValueOrDefault("target", drive.target);
                drive.targetVelocity = driveJSON.ValueOrDefault("target_velocity", drive.targetVelocity);
                UnityArticulationBody.yDrive = drive;
            }

            if (json.ContainsKey("z_drive")) {
                JObject driveJSON = json["z_drive"].Value<JObject>();
                ArticulationDrive drive = UnityArticulationBody.xDrive;
                drive.damping = driveJSON.ValueOrDefault("damping", drive.damping);
                drive.forceLimit = driveJSON.ValueOrDefault("force_limit", drive.forceLimit);
                drive.lowerLimit = driveJSON.ValueOrDefault("lower_limit", drive.lowerLimit);
                drive.upperLimit = driveJSON.ValueOrDefault("upper_limit", drive.upperLimit);
                drive.stiffness = driveJSON.ValueOrDefault("stiffness", drive.stiffness);
                drive.target = driveJSON.ValueOrDefault("target", drive.target);
                drive.targetVelocity = driveJSON.ValueOrDefault("target_velocity", drive.targetVelocity);
                UnityArticulationBody.zDrive = drive;
            }

        }
        #endregion

    }

}
