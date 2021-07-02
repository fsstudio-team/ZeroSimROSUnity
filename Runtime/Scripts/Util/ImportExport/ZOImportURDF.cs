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

        public static ZOSimDocumentRoot Import(string urdfFilePath) {
            using (StreamReader streamReader = new StreamReader(urdfFilePath)) {
                XmlDocument xmlDocument = new XmlDocument();
                xmlDocument.Load(streamReader);
                return Import(xmlDocument);
            }
        }

        public static ZOSimDocumentRoot Import(XmlDocument xmlDocument) {
            XmlNode robot = xmlDocument.GetChildByName("robot");

            GameObject rootObject = new GameObject(robot.Name);

            ZOSimDocumentRoot simDocumentRoot = rootObject.AddComponent<ZOSimDocumentRoot>();

            // get the joints & links
            XmlNode[] xmlLinks = robot.GetChildrenByName("link");
            Dictionary<string, Tuple<XmlNode, GameObject>> goLinks = new Dictionary<string, Tuple<XmlNode, GameObject>>();

            // find root link. which is a link without an explicit parent

            // create links
            foreach (XmlNode xmlLink in xmlLinks) {

                string linkName = xmlLink.Attributes["name"].Value;
                GameObject goLink = new GameObject(linkName);
                ZOSimOccurrence occurrence = goLink.AddComponent<ZOSimOccurrence>();

                GameObject goVisualsEmpty = new GameObject("visuals");
                goVisualsEmpty.transform.SetParent(goLink.transform);
                
                GameObject goCollisionsEmpty = new GameObject("collisions");
                goCollisionsEmpty.transform.SetParent(goLink.transform);

                goLinks[linkName] = new Tuple<XmlNode, GameObject>(xmlLink, goLink);
                // // process the visuals
                // XmlNode[] visuals = xmlLink.GetChildrenByName("visual");

                // foreach (XmlNode visual in visuals) {

                // }

                // get the origin
                // XmlNode origin = 

            }

            // create hierarchy from joints
            XmlNode[] xmlJoints = robot.GetChildrenByName("joint");
            foreach (var goLink in goLinks) {

                // find link parent
                GameObject linkParent = rootObject;
                foreach (XmlNode joint in xmlJoints) {
                    XmlNode xmlChildLink = joint.GetChildByName("child");
                    string childName = xmlChildLink.Attributes["link"].Value;

                    if (goLink.Value.Item2.name == childName) {
                        XmlNode xmlParentLink = joint.GetChildByName("parent");
                        string parentName = xmlParentLink.Attributes["link"].Value;
                        linkParent = goLinks[parentName].Item2;
                        break;
                    }

                }

                goLink.Value.Item2.transform.SetParent(linkParent.transform);
            }

            return simDocumentRoot;
        }

    }
}