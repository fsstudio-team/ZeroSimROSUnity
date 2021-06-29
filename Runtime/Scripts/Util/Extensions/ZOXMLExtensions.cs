using System.Xml;
using System.Collections.Generic;

namespace ZO.Util.Extensions {
    public static class ZOXMLExtensions {
        static public XmlNode GetChildByName(this XmlNode xmlNode, string name) {
            foreach (XmlNode child in xmlNode) {
                if (child.Name == name) {
                    return child;
                }
            }

            return null;
        }

        public static XmlNode[] GetChildrenByName(this XmlNode xmlNode, string name, bool recursive = false) {

            List<XmlNode> children = new List<XmlNode>();
            foreach (XmlNode child in xmlNode.ChildNodes) {
                if (child.Name == name) {
                    children.Add(child);
                }

                if (recursive) {
                    XmlNode[] recursiveChildren = child.GetChildrenByName(name, recursive); 
                    foreach (XmlNode childrensChild in recursiveChildren) {
                        children.Add(childrensChild);
                    }
                }
            }

            return children.ToArray();

        }


    }
}