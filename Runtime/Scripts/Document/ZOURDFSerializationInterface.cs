using System.Xml.Linq;
using System.Collections.Generic;

namespace ZO.Document {

    /// <summary>
    /// Defines a standard interface to serialize and deserialize ZOSim objects.
    /// </summary>
    public interface ZOURDFSerializationInterface {

        /// <summary>
        /// Property for getting the XML of this object.
        /// <see>Serialize & Deserialize</see>
        /// </summary>
        /// <value></value>
        XElement XML { get; }

        /// <summary>
        /// Unique name
        /// </summary>
        /// <value></value>
        string Name { get; }

        /// <summary>
        /// Type of this object.  For example "joint.hinge".
        /// </summary>
        /// <value></value>
        string Type { get; }


        void BuildLinks(ZOSimDocumentRoot documentRoot, XElement robot, ZOSimOccurrence parent = null);
        void BuildJoints(ZOSimDocumentRoot documentRoot, XElement robot, ZOSimOccurrence parent = null);

        /// <summary>
        /// Sets the state of the object from XML. XML could come from file or network or wherever.
        /// </summary>
        /// <param name="json"></param>
        void Deserialize(ZOSimDocumentRoot documentRoot, XElement xml);
        
    }

}