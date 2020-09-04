using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Newtonsoft.Json.Linq;

namespace ZO.Document {

    /// <summary>
    /// Defines a standard interface to serialize and deserialize ZOSim objects.
    /// </summary>
    public interface ZOSerializationInterface {

        /// <summary>
        /// Property for getting the JSON of this object.
        /// <see>Serialize & Deserialize</see>
        /// </summary>
        /// <value></value>
        JObject JSON { get; }

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


        /// <summary>
        /// From the current state of the object it will generate the JSON for this object and any children.
        /// </summary>
        /// <param name="parent"></param>
        /// <returns></returns>
        JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null);

        /// <summary>
        /// Sets the state of the object from JSON. JSON could come from file or network or wherever.
        /// </summary>
        /// <param name="json"></param>
        void Deserialize(ZOSimDocumentRoot documentRoot, JObject json);
        
    }
}