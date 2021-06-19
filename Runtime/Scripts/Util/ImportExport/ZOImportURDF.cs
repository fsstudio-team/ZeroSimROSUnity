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
using System.Xml;

namespace ZO.ImportExport {


    public class ZOImportURDF {

        public ZOSimDocumentRoot Import(string urdfFilePath) {
            using(StreamReader streamReader = new StreamReader(urdfFilePath)) {
                XmlDocument xmlDocument = new XmlDocument();
                xmlDocument.Load(streamReader);
                return Import(xmlDocument);
            }
        }

        public ZOSimDocumentRoot Import(XmlDocument xmlDocument) {                        
            XmlNode robot = xmlDocument.GetChildByName("robot");

            GameObject rootObject = new GameObject(robot.Name);            

            ZOSimDocumentRoot simDocumentRoot = rootObject.AddComponent<ZOSimDocumentRoot>();


            // create the URDF links in Unity 
            XmlNode[] links = robot.GetChildrenByName("link");
            foreach(XmlNode link in links) {
                // process the visuals
                XmlNode[] visuals = link.GetChildrenByName("visual");

            }

            return simDocumentRoot;
        }

    }
}