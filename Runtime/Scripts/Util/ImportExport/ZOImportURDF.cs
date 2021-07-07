using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System;
using System.Text;
using ZO.Document;
using ZO.Physics;
using ZO.Math;
using ZO.Util.Extensions;
using System.Xml.Linq;
using System.Xml;

namespace ZO.ImportExport {


    public class ZOImportURDF {

        public static string WorkingDirectory {
            get; set;
        } = ".";

        public static ZOSimDocumentRoot Import(string urdfFilePath) {
            using (StreamReader streamReader = new StreamReader(urdfFilePath)) {
                WorkingDirectory = Path.GetDirectoryName(urdfFilePath);
                XmlDocument xmlDocument = new XmlDocument();
                xmlDocument.Load(streamReader);
                return Import(xmlDocument);
            }
        }

        public static ZOSimDocumentRoot Import(XmlDocument xmlDocument) {
            XmlNode robot = xmlDocument.GetChildByName("robot");

            GameObject rootObject = new GameObject(robot.Attributes["name"].Value);

            ZOSimDocumentRoot simDocumentRoot = rootObject.AddComponent<ZOSimDocumentRoot>();

            // get the joints & links
            XmlNode[] xmlLinks = robot.GetChildrenByName("link");
            Dictionary<string, Tuple<XmlNode, GameObject>> goLinks = new Dictionary<string, Tuple<XmlNode, GameObject>>();

            // find root link. which is a link without an explicit parent

            // create links
            foreach (XmlNode xmlLink in xmlLinks) {

                string linkName = xmlLink.Attributes["name"].Value;
                GameObject goLink = new GameObject(linkName);

                GameObject goVisualsEmpty = new GameObject("visuals");
                goVisualsEmpty.transform.SetParent(goLink.transform);

                GameObject goCollisionsEmpty = new GameObject("collisions");
                goCollisionsEmpty.transform.SetParent(goLink.transform);

                goLinks[linkName] = new Tuple<XmlNode, GameObject>(xmlLink, goLink);

                // process the visuals
                XmlNode[] xmlVisuals = xmlLink.GetChildrenByName("visual");

                foreach (XmlNode xmlVisual in xmlVisuals) {


                    XmlNode[] xmlGeometries = xmlVisual.GetChildrenByName("geometry");

                    foreach (XmlNode xmlGeom in xmlGeometries) {

                        GameObject visualGeo = null;

                        XmlNode xmlBox = xmlGeom.GetChildByName("box");
                        if (xmlBox != null) {
                            visualGeo = GameObject.CreatePrimitive(PrimitiveType.Cube);
                            Vector3 size = xmlBox.Attributes["size"].Value.FromURDFStringToVector3();

                            visualGeo.transform.localScale = size.Ros2UnityScale();
                        }

                        XmlNode xmlCylinder = xmlGeom.GetChildByName("cylinder");
                        if (xmlCylinder != null) {
                            visualGeo = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                            float radius = float.Parse(xmlCylinder.Attributes["radius"].Value);
                            float length = float.Parse(xmlCylinder.Attributes["length"].Value);
                            visualGeo.transform.localScale = new Vector3(radius * 2.0f, length * 0.5f, radius * 2.0f);
                        }

                        XmlNode xmlSphere = xmlGeom.GetChildByName("sphere");
                        if (xmlSphere != null) {
                            visualGeo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                            float radius = float.Parse(xmlSphere.Attributes["radius"].Value);
                            visualGeo.transform.localScale = new Vector3(radius * 2.0f, radius * 2.0f, radius * 2.0f);
                        }

                        XmlNode xmlMesh = xmlGeom.GetChildByName("mesh");
                        if (xmlMesh != null) {
                            // TODO
                        }
                        if (visualGeo != null) {
                            // set parent
                            visualGeo.transform.SetParent(goVisualsEmpty.transform);
                            string visualName = xmlVisual.Attributes["name"].Value;
                            if (visualName != null) {
                                visualGeo.name = visualName;
                            }

                            // set transform
                            XmlNode xmlOrigin = xmlVisual.GetChildByName("origin");
                            Tuple<Vector3, Quaternion> transform = OriginXMLToUnity(xmlOrigin);
                            visualGeo.transform.localPosition = transform.Item1;
                            visualGeo.transform.localRotation = transform.Item2;

                        }

                    }

                }

                // get the origin
                // XmlNode origin = 

            }

            // create hierarchy from joints
            XmlNode[] xmlJoints = robot.GetChildrenByName("joint");
            foreach (var goLink in goLinks) {

                // find link parent
                GameObject linkParent = rootObject;
                GameObject linkChild = goLink.Value.Item2;
                XmlNode workingJoint = null;
                foreach (XmlNode joint in xmlJoints) {
                    XmlNode xmlChildLink = joint.GetChildByName("child");
                    string childName = xmlChildLink.Attributes["link"].Value;

                    if (goLink.Value.Item2.name == childName) {
                        XmlNode xmlParentLink = joint.GetChildByName("parent");
                        string parentName = xmlParentLink.Attributes["link"].Value;
                        linkParent = goLinks[parentName].Item2;

                        workingJoint = joint;

                        break;
                    }

                }

                linkChild.transform.SetParent(linkParent.transform);
                ZOSimOccurrence occurrence = linkChild.AddComponent<ZOSimOccurrence>();

                // set transform
                if (workingJoint != null) {
                    // set the transform
                    XmlNode xmlOrigin = workingJoint.GetChildByName("origin");
                    Tuple<Vector3, Quaternion> transform = OriginXMLToUnity(xmlOrigin);
                    linkChild.transform.localPosition = transform.Item1;
                    linkChild.transform.localRotation = transform.Item2;

                    // add rigidbody components
                    Rigidbody childRigidBody = linkChild.AddComponent<Rigidbody>();
                    Rigidbody parentRigidBody = linkParent.AddComponent<Rigidbody>();

                    // add joints
                    string jointType = workingJoint.Attributes["type"].Value;
                    if (jointType == "revolute") {

                        ZOHingeJoint hingeJoint = linkParent.AddComponent<ZOHingeJoint>();
                        hingeJoint.ConnectedBody = childRigidBody;
                        hingeJoint.Anchor = transform.Item1;

                        XmlNode xmlAxis = workingJoint.GetChildByName("axis");
                        hingeJoint.Axis = xmlAxis.Attributes["xyz"].Value.FromURDFStringToVector3().Ros2Unity();
                    }

                }


            }

            return simDocumentRoot;
        }

        protected static Tuple<Vector3, Quaternion> OriginXMLToUnity(XmlNode xmlOrigin) {
            Vector3 translation = Vector3.zero;
            string xyz = xmlOrigin.Attributes["xyz"].Value;
            if (xyz != null && xyz != "") {
                translation = xyz.FromURDFStringToVector3().Ros2Unity();
            }

            Quaternion rotation = Quaternion.identity;
            string rpy = xmlOrigin.Attributes["rpy"].Value;
            if (rpy != null && rpy != "") {
                rotation = rpy.FromURDFStringToVector3().RosRollPitchYawToQuaternion();
            }
            Tuple<Vector3, Quaternion> transform = new Tuple<Vector3, Quaternion>(translation, rotation);

            return transform;
        }

    }
}