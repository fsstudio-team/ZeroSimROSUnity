using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using ZO.Document;
using ZO.Physics;
using ZO.Math;
using ZO.Util.Extensions;
using System.Xml.Linq;

namespace ZO.ImportExport {


    public class ZOExportURDF {

        protected readonly struct URDFJoint {
            public URDFJoint(ZOSimOccurrence child, ZOSimOccurrence parent, Vector3 anchor, Vector3 connectedAnchor) {
                Child = child;
                Parent = parent;
                Anchor = anchor;
                ConnectedAnchor = connectedAnchor;
            }


            public ZOSimOccurrence Child {
                get;
            }

            public ZOSimOccurrence Parent {
                get;
            }

            public Vector3 Anchor {
                get;
            }

            public Vector3 ConnectedAnchor {
                get;
            }

            public static bool operator ==(URDFJoint a, URDFJoint b) {
                return a.Equals(b);
            }
            public static bool operator !=(URDFJoint a, URDFJoint b) {
                return !a.Equals(b);
            }

        }

        /// <summary>
        /// Recursively build a URDF xml document given the base/root `ZOSimOccurrence`.
        /// Really should only be used by `ZODocumentRoot`.  See: `ZODocumentRoot.ExportURDF`.
        /// </summary>
        /// <param name="robot">The robot XML document</param>
        /// <param name="simOccurrence">The root `ZOSimOccurence`</param>
        /// <param name="baseTransform">The base relative transform, usually the transform of the `ZODocumentRoot`</param>
        public void BuildURDF(XElement robot, ZOSimOccurrence simOccurrence, Matrix4x4 baseTransform) {

            List<URDFJoint> joints = new List<URDFJoint>();

            BuildURDFJoints(robot, null, simOccurrence, baseTransform, ref joints);


            // build links
            HashSet<ZOSimOccurrence> links = new HashSet<ZOSimOccurrence>();
            foreach (URDFJoint joint in joints) {
                
                if (links.Contains(joint.Parent) == false) { // build parent link if not exist
                    Vector3 offset = -1.0f * joint.ConnectedAnchor;
                    if (joints[0] == joint) {  // if base joint do not apply any offset to parent link
                        offset = Vector3.zero;
                    }
                    joint.Parent.BuildURDFLink(robot, offset);
                    links.Add(joint.Parent);
                }
                if (links.Contains(joint.Child) == false) { // build child link if not exist
                    Vector3 offset = -1.0f * joint.ConnectedAnchor;
                    joint.Child.BuildURDFLink(robot, offset);
                    links.Add(joint.Child);
                }
            }

            if (joints.Count == 0) { // if we don't have any joints then create a dummy world link and joint to first link

                XElement link = new XElement("link");
                link.SetAttributeValue("name", "World");
                robot.Add(link);

                simOccurrence.BuildURDFLink(robot, Vector3.zero);

                XElement jointX = new XElement("joint");
                jointX.SetAttributeValue("name", $"World_to_{simOccurrence.Name}");
                jointX.SetAttributeValue("type", "fixed");

                XElement parentX = new XElement("parent");
                parentX.SetAttributeValue("link", "World");
                jointX.Add(parentX);

                XElement childX = new XElement("child");
                childX.SetAttributeValue("link", simOccurrence.Name);
                jointX.Add(childX);
                robot.Add(jointX);

            }

        }

        protected void BuildURDFJoints(XElement robot, ZOSimOccurrence parent, ZOSimOccurrence child, Matrix4x4 worldJointMatrix, ref List<URDFJoint> joints) {

            // if we have a parent build a joint
            if (parent) {
                XElement jointX = new XElement("joint");
                jointX.SetAttributeValue("name", $"{parent.Name}_to_{child.Name}");
                // Transform jointTransform = this.transform;
                Matrix4x4 jointMatrix = Matrix4x4.identity;

                ZOHingeJoint hingeJoint = parent.GetComponent<ZOHingeJoint>();
                if (hingeJoint != null) {
                    jointX.SetAttributeValue("type", "revolute");

                    // create axis
                    Vector3 axis = hingeJoint.UnityHingeJoint.axis.Unity2Ros();
                    XElement axisX = new XElement("axis");
                    axisX.SetAttributeValue("xyz", axis.ToXMLString());
                    jointX.Add(axisX);

                    // create limits
                    // TODO:
                    XElement limitX = new XElement("limit");
                    limitX.SetAttributeValue("effort", 10000f); // HACK
                    limitX.SetAttributeValue("velocity", 3.14f); // HACK
                    jointX.Add(limitX);

                    // Add the anchor position
                    jointMatrix = parent.transform.WorldTranslationRotationMatrix();
                    jointMatrix = jointMatrix.AddTranslation(hingeJoint.Anchor);

                    // save this off as the new world joint matrix
                    Matrix4x4 newWorldJointMatrix = jointMatrix;

                    // subtract out the parent root
                    jointMatrix = jointMatrix * worldJointMatrix.inverse;
                    worldJointMatrix = newWorldJointMatrix;

                    Vector3 xyz = jointMatrix.Position().Unity2Ros();
                    Vector3 rpy = jointMatrix.rotation.Unity2RosRollPitchYaw();

                    XElement origin = new XElement("origin");
                    origin.SetAttributeValue("xyz", xyz.ToXMLString());
                    origin.SetAttributeValue("rpy", rpy.ToXMLString());
                    jointX.Add(origin);

                    URDFJoint joint = new URDFJoint(child, parent, hingeJoint.Anchor, hingeJoint.ConnectedAnchor);
                    joints.Add(joint);


                } else { // children of the parent even without an explicit joint are "fixed" joints

                    jointX.SetAttributeValue("type", "fixed");
                    jointMatrix = parent.transform.WorldTranslationRotationMatrix().inverse * child.transform.WorldTranslationRotationMatrix();

                    Vector3 xyz = jointMatrix.Position().Unity2Ros();
                    Vector3 rpy = jointMatrix.rotation.Unity2RosRollPitchYaw();

                    XElement origin = new XElement("origin");
                    origin.SetAttributeValue("xyz", xyz.ToXMLString());
                    origin.SetAttributeValue("rpy", rpy.ToXMLString());
                    jointX.Add(origin);

                    URDFJoint joint = new URDFJoint(child, parent, jointMatrix.Position(), Vector3.zero);
                    joints.Add(joint);


                }


                robot.Add(jointX);

                XElement parentX = new XElement("parent");
                parentX.SetAttributeValue("link", parent.Name);
                jointX.Add(parentX);

                XElement childX = new XElement("child");
                childX.SetAttributeValue("link", child.Name);
                jointX.Add(childX);

            }
            // recursively go through the children
            foreach (Transform c in child.transform) {
                ZOSimOccurrence childOfChild = c.GetComponent<ZOSimOccurrence>();
                if (childOfChild) {
                    BuildURDFJoints(robot, child, childOfChild, worldJointMatrix, ref joints);
                }
            }

        }



    }

}