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
        public XDocument XML {
            get; set;
        } = new XDocument();

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

        private List<Transform> _visualMeshesToExport = new List<Transform>();
        public List<Transform> VisualMeshesToExport {
            get { return _visualMeshesToExport; }
        }

        public struct CollisionMesh {
            public Mesh mesh;
            public Vector3 scale;
        }
        private List<CollisionMesh> _collisionMeshesToExport = new List<CollisionMesh>();


        public List<CollisionMesh> CollisionMeshesToExport {
            get { return _collisionMeshesToExport; }
        }


        public static void ExportToDirectory(ZOSimDocumentRoot documentRoot, string directoryPath) {
            ZOExportURDF exportURDF = new ZOExportURDF();
            XDocument urdfXML = exportURDF.BuildURDF(documentRoot);
            string urdfFilePath = Path.Combine(directoryPath, $"{documentRoot.Name}.urdf");
            urdfXML.Save(urdfFilePath);

            // save out visual and collision meshes
            foreach (Transform meshTransform in exportURDF.VisualMeshesToExport) {
                ZOExportOBJ exportOBJ = new ZOExportOBJ();
                exportOBJ.ExportToDirectory(meshTransform.gameObject, directoryPath, true, false, ZOExportOBJ.Orientation.URDF);
            }

            foreach (CollisionMesh collisionMesh in exportURDF.CollisionMeshesToExport) {
                ZOExportOBJ exportOBJ = new ZOExportOBJ();
                string collisionMeshFilePath = Path.Combine(directoryPath, $"{collisionMesh.mesh.name}_collider.obj");
                exportOBJ.ExportMesh(collisionMesh.mesh, collisionMeshFilePath, ZOExportOBJ.Orientation.URDF, collisionMesh.scale);
            }

            Debug.Log($"INFO: ZOExportURDF Saved URDF: {urdfFilePath}");

        }


        /// <summary>
        /// Recursively build a URDF xml document given the base/root `ZOSimOccurrence`.
        /// Really should only be used by `ZODocumentRoot`.  See: `ZODocumentRoot.ExportURDF`.
        /// </summary>
        /// <param name="robot">The robot XML document</param>
        /// <param name="simOccurrence">The root `ZOSimOccurence`</param>
        /// <param name="baseTransform">The base relative transform, usually the transform of the `ZODocumentRoot`</param>
        public XDocument BuildURDF(ZOSimDocumentRoot documentRoot) {

            // create root document and robot element
            XElement robot = new XElement("robot");
            robot.SetAttributeValue("name", documentRoot.Name);
            XML = new XDocument(robot);

            // go through the ZOSimOccurrences and convert into URDF Links and Joints    
            ZOSimOccurrence simOccurrence = null;
            foreach (Transform child in documentRoot.transform) {  // BUG: Should only ever be one base object for URDF!!!
                simOccurrence = child.GetComponent<ZOSimOccurrence>();
                if (simOccurrence) {
                    break;  // BUG: stops at first sim occurrence
                }
            }

            Matrix4x4 baseTransform = documentRoot.transform.WorldTranslationRotationMatrix();


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
                    BuildLink(joint.Parent, robot, offset);
                    links.Add(joint.Parent);
                }
                if (links.Contains(joint.Child) == false) { // build child link if not exist
                    Vector3 offset = -1.0f * joint.ConnectedAnchor;
                    BuildLink(joint.Child, robot, offset);
                    links.Add(joint.Child);
                }
            }

            if (joints.Count == 0) { // if we don't have any joints then create a dummy world link and joint to first link

                XElement link = new XElement("link");
                link.SetAttributeValue("name", "World");
                robot.Add(link);

                BuildLink(simOccurrence, robot, Vector3.zero);

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

            return XML;

        }

        protected void BuildURDFJoints(XElement robot, ZOSimOccurrence parent, ZOSimOccurrence child, Matrix4x4 worldJointMatrix, ref List<URDFJoint> joints) {

            // if we have a parent build a joint
            if (parent) {

                ZOJointInterface[] zoJoints = parent.GetComponents<ZOJointInterface>();

                // even if there are no explicit fixed joint children of the parent are "automatically" set as fixed.
                if (zoJoints.Length == 0) {
                    XElement jointX = new XElement("joint");
                    jointX.SetAttributeValue("name", $"Fixed_{parent.Name}_to_{child.Name}");
                    // Transform jointTransform = this.transform;
                    Matrix4x4 jointMatrix = Matrix4x4.identity;
                    jointX.SetAttributeValue("type", "fixed");
                    jointMatrix = parent.transform.WorldTranslationRotationMatrix().inverse * child.transform.WorldTranslationRotationMatrix();

                    Vector3 xyz = jointMatrix.Position().Unity2Ros();
                    Vector3 rpy = jointMatrix.rotation.Unity2RosRollPitchYaw();

                    XElement origin = new XElement("origin");
                    origin.SetAttributeValue("xyz", xyz.ToXMLString());
                    origin.SetAttributeValue("rpy", rpy.ToXMLString());
                    jointX.Add(origin);

                    URDFJoint j = new URDFJoint(child, parent, jointMatrix.Position(), Vector3.zero);
                    joints.Add(j);

                    robot.Add(jointX);

                    XElement parentX = new XElement("parent");
                    parentX.SetAttributeValue("link", parent.Name);
                    jointX.Add(parentX);

                    XElement childX = new XElement("child");
                    childX.SetAttributeValue("link", child.Name);
                    jointX.Add(childX);


                }

                foreach (ZOJointInterface joint in zoJoints) {
                    if (joint.ConnectedOccurrence == child) {  // only if this joint is pointing at us
                        XElement jointX = new XElement("joint");
                        jointX.SetAttributeValue("name", $"{joint.Type}_{parent.Name}_to_{child.Name}");
                        // Transform jointTransform = this.transform;
                        Matrix4x4 jointMatrix = Matrix4x4.identity;


                        if (joint.GetType() == typeof(ZOHingeJoint)) {
                            // TODO: distinguish between revolute (a hinge with limits) vs continuous (joint with no limits)
                            ZOHingeJoint hingeJoint = joint as ZOHingeJoint;
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
                            //jointMatrix = jointMatrix * worldJointMatrix.inverse;
                            jointMatrix = worldJointMatrix.inverse * jointMatrix;
                            worldJointMatrix = newWorldJointMatrix;

                            Vector3 xyz = jointMatrix.Position().Unity2Ros();
                            Vector3 rpy = jointMatrix.rotation.Unity2RosRollPitchYaw();

                            XElement origin = new XElement("origin");
                            origin.SetAttributeValue("xyz", xyz.ToXMLString());
                            origin.SetAttributeValue("rpy", rpy.ToXMLString());
                            jointX.Add(origin);

                            URDFJoint j = new URDFJoint(child, parent, hingeJoint.Anchor, hingeJoint.ConnectedAnchor);
                            joints.Add(j);


                        } else if (joint.GetType() == typeof(ZOPrismaticJoint)) {
                            ZOPrismaticJoint prismaticJoint = joint as ZOPrismaticJoint;
                            jointX.SetAttributeValue("type", "prismatic");

                            // create axis
                            Vector3 axis = prismaticJoint.Axis.Unity2Ros();
                            XElement axisX = new XElement("axis");
                            axisX.SetAttributeValue("xyz", axis.ToXMLString());
                            jointX.Add(axisX);

                            // create limits
                            // TODO:
                            XElement limitX = new XElement("limit");
                            limitX.SetAttributeValue("lower", -prismaticJoint.UnityConfigurableJoint.linearLimit.limit);  //HACK: quick hack to get the "correct" limit
                            limitX.SetAttributeValue("upper", prismaticJoint.UnityConfigurableJoint.linearLimit.limit);   //HACK: quick hack to get the "correct" limit

                            // limitX.SetAttributeValue("lower", prismaticJoint.JointLimits.LowerLimit);
                            // limitX.SetAttributeValue("upper", prismaticJoint.JointLimits.UpperLimit);
                            limitX.SetAttributeValue("effort", 10000f); // HACK
                            limitX.SetAttributeValue("velocity", 10000f); // HACK
                            jointX.Add(limitX);

                            // Add the anchor position
                            // Vector3 worldAnchor = prismaticJoint.transform.TransformPoint(prismaticJoint.Anchor);
                            // Vector3 localAnchor = worldJointMatrix.inverse.MultiplyPoint(worldAnchor);
                            // worldJointMatrix = parent.transform.WorldTranslationRotationMatrix();
                            // Vector3 xyz = prismaticJoint.Anchor.Unity2Ros();//localAnchor.Unity2Ros();
                            // Vector3 rpy = Vector3.zero; // BUGBUG
                            jointMatrix = prismaticJoint.transform.WorldTranslationRotationMatrix();
                            jointMatrix = jointMatrix.AddTranslation(prismaticJoint.Anchor);

                            // TODO: the links should be offset by the connected anchor.  Not the way done below.
                            Vector3 worldConnectedAnchor = prismaticJoint.ConnectedBody.transform.TransformPoint(prismaticJoint.ConnectedAnchor);
                            Vector3 relativeToParentConnectedAnchor = prismaticJoint.transform.InverseTransformPoint(worldConnectedAnchor);
                            // jointMatrix = jointMatrix.AddTranslation(relativeToParentConnectedAnchor);

                            // save this off as the new world joint matrix
                            Matrix4x4 newWorldJointMatrix = jointMatrix;

                            // subtract out the parent root
                            // jointMatrix = jointMatrix * worldJointMatrix.inverse;
                            jointMatrix = worldJointMatrix.inverse * jointMatrix;

                            worldJointMatrix = newWorldJointMatrix;

                            Vector3 xyz = jointMatrix.Position().Unity2Ros();
                            Vector3 rpy = jointMatrix.rotation.Unity2RosRollPitchYaw();

                            XElement origin = new XElement("origin");
                            origin.SetAttributeValue("xyz", xyz.ToXMLString());
                            origin.SetAttributeValue("rpy", rpy.ToXMLString());
                            jointX.Add(origin);

                            URDFJoint j = new URDFJoint(child, parent, prismaticJoint.Anchor, prismaticJoint.ConnectedAnchor);
                            // URDFJoint j = new URDFJoint(child, parent, prismaticJoint.Anchor, Vector3.zero);
                            joints.Add(j);

                        } else { // children of the parent even without an explicit joint are "fixed" joints

                            jointX.SetAttributeValue("type", "fixed");
                            jointMatrix = parent.transform.WorldTranslationRotationMatrix().inverse * child.transform.WorldTranslationRotationMatrix();

                            Vector3 xyz = jointMatrix.Position().Unity2Ros();
                            Vector3 rpy = jointMatrix.rotation.Unity2RosRollPitchYaw();

                            XElement origin = new XElement("origin");
                            origin.SetAttributeValue("xyz", xyz.ToXMLString());
                            origin.SetAttributeValue("rpy", rpy.ToXMLString());
                            jointX.Add(origin);

                            URDFJoint j = new URDFJoint(child, parent, jointMatrix.Position(), Vector3.zero);
                            joints.Add(j);


                        }


                        robot.Add(jointX);

                        XElement parentX = new XElement("parent");
                        parentX.SetAttributeValue("link", parent.Name);
                        jointX.Add(parentX);

                        XElement childX = new XElement("child");
                        childX.SetAttributeValue("link", child.Name);
                        jointX.Add(childX);



                    }
                }

            }
            // recursively go through the children
            foreach (Transform c in child.transform) {
                ZOSimOccurrence childOfChild = c.GetComponent<ZOSimOccurrence>();
                if (childOfChild) {
                    BuildURDFJoints(robot, child, childOfChild, worldJointMatrix, ref joints);
                }
            }
        }

        /// <summary>
        /// URDF has a "flattened" hierarchy where the hierarchy is build by URDF joints.
        /// </summary>
        protected void BuildLink(ZOSimOccurrence simOccurrence, XElement robot, Vector3 anchorOffset) {

            XElement link = new XElement("link");
            link.SetAttributeValue("name", simOccurrence.Name);
            robot.Add(link);

            // build inertial
            BuildLinkInertial(simOccurrence, link);


            // build the visual elements
            // go through the children
            foreach (Transform child in simOccurrence.transform) {

                // check for any visuals
                // NOTE: a gameobject named visuals is treated as a special container of visual objects
                if (child.name.ToLower() == "visuals") {
                    // go through the children of the visuals and get all the models
                    foreach (Transform visualsChild in child) {
                        // check if it is a primitive type (cube, sphere, cylinder, etc)
                        BuildLinkVisuals(visualsChild, link, anchorOffset);

                        // we will do any collider that are attached to the visual
                        BuildLinkCollisions(visualsChild, link, anchorOffset);
                    }
                }

                // build collisions
                if (child.name.ToLower() == "collisions") {
                    // go through the children of the collisions and get all the models
                    foreach (Transform collisionChild in child) {
                        BuildLinkCollisions(collisionChild, link, anchorOffset);
                    }
                }
            }
        }


        protected void BuildLinkCollisions(Transform collisionTransform, XElement link, Vector3 anchorOffset) {
            Collider[] colliders = collisionTransform.GetComponentsInChildren<Collider>();
            foreach (Collider collider in colliders) {
                XElement collision = new XElement("collision");
                collision.SetAttributeValue("name", collisionTransform.name);
                XElement geometry = new XElement("geometry");
                Vector3 center = Vector3.zero;

                if (collider.GetType() == typeof(BoxCollider)) {
                    BoxCollider boxCollider = collider as BoxCollider;
                    XElement box = new XElement("box");

                    Vector3 boxSize = new Vector3(boxCollider.size.x * collider.transform.localScale.x,
                                                    boxCollider.size.y * collider.transform.localScale.y,
                                                    boxCollider.size.z * collider.transform.localScale.z);
                    box.SetAttributeValue("size", boxSize.Unity2RosScale().ToXMLString());
                    geometry.Add(box);

                    center = new Vector3(boxCollider.center.x * boxCollider.transform.localScale.x,
                                        boxCollider.center.y * boxCollider.transform.localScale.y,
                                        boxCollider.center.z * boxCollider.transform.localScale.z);
                } else if (collider.GetType() == typeof(SphereCollider)) {
                    SphereCollider sphereCollider = collider as SphereCollider;
                    XElement sphere = new XElement("sphere");
                    float radius = sphereCollider.radius * collider.transform.localScale.x;
                    sphere.SetAttributeValue("radius", radius);
                    geometry.Add(sphere);

                    center = new Vector3(sphereCollider.center.x * sphereCollider.transform.localScale.x,
                                        sphereCollider.center.y * sphereCollider.transform.localScale.y,
                                        sphereCollider.center.z * sphereCollider.transform.localScale.z);


                } else if (collider.GetType() == typeof(CapsuleCollider)) {
                    CapsuleCollider capsuleCollider = collider as CapsuleCollider;
                    XElement cylinder = new XElement("cylinder");
                    float radius = capsuleCollider.radius * collider.transform.localScale.x;
                    float height = capsuleCollider.height * collider.transform.localScale.y;
                    cylinder.SetAttributeValue("radius", radius);
                    cylinder.SetAttributeValue("length", height);
                    geometry.Add(cylinder);

                    center = new Vector3(capsuleCollider.center.x * capsuleCollider.transform.localScale.x,
                                        capsuleCollider.center.y * capsuleCollider.transform.localScale.y,
                                        capsuleCollider.center.z * capsuleCollider.transform.localScale.z);


                } else if (collider.GetType() == typeof(MeshCollider)) {
                    MeshCollider meshCollider = collider as MeshCollider;
                    CollisionMesh collisionMesh = new CollisionMesh {
                        mesh = meshCollider.sharedMesh,
                        scale = collisionTransform.lossyScale
                    };
                    _collisionMeshesToExport.Add(collisionMesh);

                    XElement mesh = new XElement("mesh");
                    mesh.SetAttributeValue("filename", $"{meshCollider.sharedMesh.name}_collider.obj");
                    Vector3 scale = collisionTransform.localScale;
                    mesh.SetAttributeValue("scale", scale.ToXMLString());
                    geometry.Add(mesh);

                }


                if (geometry.HasElements) {
                    // build origin
                    Vector3 position = collisionTransform.localPosition + anchorOffset + center;
                    Vector3 xyz = position.Unity2Ros();
                    Vector3 rpy = new Vector3(-collisionTransform.localEulerAngles.z * Mathf.Deg2Rad,
                                                collisionTransform.localEulerAngles.x * Mathf.Deg2Rad,
                                                -collisionTransform.localEulerAngles.y * Mathf.Deg2Rad);

                    XElement origin = new XElement("origin");
                    origin.SetAttributeValue("xyz", xyz.ToXMLString());
                    origin.SetAttributeValue("rpy", rpy.ToXMLString());

                    collision.Add(origin);

                    collision.Add(geometry);
                    link.Add(collision);
                }

            }
        }

        protected void BuildLinkVisuals(Transform visualTransform, XElement link, Vector3 anchorOffset) {
            // build 3d primitive if exists
            // Check children
            MeshFilter[] meshFilters = visualTransform.GetComponentsInChildren<MeshFilter>();
            foreach (MeshFilter meshFilter in meshFilters) {
                if (meshFilter) {

                    MeshRenderer meshRenderer = visualTransform.GetComponent<MeshRenderer>();
                    Collider collider = null;
                    XElement visual = new XElement("visual");
                    visual.SetAttributeValue("name", visualTransform.name);
                    XElement geometry = new XElement("geometry");

                    if (meshFilter.sharedMesh.name.Contains("Cube")) {
                        XElement box = new XElement("box");

                        Vector3 boxSize = visualTransform.localScale.Unity2RosScale();
                        box.SetAttributeValue("size", boxSize.ToXMLString());
                        geometry.Add(box);

                    } else if (meshFilter.sharedMesh.name.Contains("Sphere")) {
                        XElement sphere = new XElement("sphere");
                        float radius = visualTransform.localScale.x / 2.0f;
                        sphere.SetAttributeValue("radius", radius);
                        geometry.Add(sphere);

                    } else if (meshFilter.sharedMesh.name.Contains("Cylinder")) {
                        XElement cylinder = new XElement("cylinder");
                        float radius = visualTransform.localScale.x / 2.0f;
                        float height = visualTransform.localScale.y * 2.0f;
                        cylinder.SetAttributeValue("radius", radius);
                        cylinder.SetAttributeValue("length", height);

                        geometry.Add(cylinder);


                    } else {  // regular mesh so export meshes as OBJ
                        XElement mesh = new XElement("mesh");
                        mesh.SetAttributeValue("filename", $"{visualTransform.name}.obj");
                        Vector3 scale = visualTransform.localScale;
                        mesh.SetAttributeValue("scale", scale.ToXMLString());
                        geometry.Add(mesh);

                        _visualMeshesToExport.Add(visualTransform);
                    }

                    if (geometry.HasElements) {
                        // build origin
                        Vector3 position = visualTransform.localPosition + anchorOffset;
                        Vector3 xyz = position.Unity2Ros();
                        Vector3 rpy = new Vector3(-visualTransform.localEulerAngles.z * Mathf.Deg2Rad,
                                                    visualTransform.localEulerAngles.x * Mathf.Deg2Rad,
                                                    -visualTransform.localEulerAngles.y * Mathf.Deg2Rad);

                        XElement origin = new XElement("origin");
                        origin.SetAttributeValue("xyz", xyz.ToXMLString());
                        origin.SetAttributeValue("rpy", rpy.ToXMLString());

                        visual.Add(origin);

                        visual.Add(geometry);
                        link.Add(visual);
                    }

                }

            }
        }

        public void BuildLinkInertial(ZOSimOccurrence simOccurrence, XElement link) {
            Rigidbody rigidbody = simOccurrence.GetComponent<Rigidbody>();
            if (rigidbody != null) {
                XElement inertial = new XElement("inertial");
                XElement mass = new XElement("mass");
                mass.SetAttributeValue("value", rigidbody.mass);
                inertial.Add(mass);

                // calculate the inertia components
                // Unity/PhysX is kinda weird in that it is represented by a diagonal vector and and rotation quaternion.
                Matrix4x4 lambdaMatrix = new Matrix4x4();
                lambdaMatrix[0, 0] = rigidbody.inertiaTensor.x;
                lambdaMatrix[1, 1] = rigidbody.inertiaTensor.y;
                lambdaMatrix[2, 2] = rigidbody.inertiaTensor.z;
                lambdaMatrix[3, 3] = 1.0f;

                Matrix4x4 qMatrix = Matrix4x4.Rotate(rigidbody.inertiaTensorRotation);
                Matrix4x4 qMatrixTransposed = qMatrix.transpose;
                Matrix4x4 inertiaMatrix = qMatrix * lambdaMatrix * qMatrixTransposed;

                float ixx = inertiaMatrix[2, 2];
                float ixy = -inertiaMatrix[0, 2];
                float ixz = inertiaMatrix[1, 2];
                float iyy = inertiaMatrix[0, 0];
                float iyz = -inertiaMatrix[0, 1];
                float izz = inertiaMatrix[1, 1];

                XElement inertia = new XElement("inertia");
                inertia.SetAttributeValue("ixx", ixx);
                inertia.SetAttributeValue("ixy", ixy);
                inertia.SetAttributeValue("ixz", ixz);
                inertia.SetAttributeValue("iyy", iyy);
                inertia.SetAttributeValue("iyz", iyz);
                inertia.SetAttributeValue("izz", izz);

                inertial.Add(inertia);

                // center of mass
                XElement origin = new XElement("origin");
                origin.SetAttributeValue("xyz", rigidbody.centerOfMass.Unity2Ros().ToXMLString());
                origin.SetAttributeValue("rpy", Vector3.zero.ToXMLString());  // BUG:  always zero?
                inertial.Add(origin);


                link.Add(inertial);
            }
        }



    }

}