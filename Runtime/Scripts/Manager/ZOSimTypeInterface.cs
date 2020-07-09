using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Newtonsoft.Json.Linq;

namespace ZO {
    interface ZOSimTypeInterface {

        /// <summary>
        /// Property for getting the JSON of this object.
        /// <see>LoadFromJSON & BuildJSON</see>
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
        /// Used for importing a ZeroSim document file.  Will import assets such as
        /// visual and collision meshes.  
        /// <see>ZOImportZeroSim</see>
        /// </summary>
        /// <param name="json"></param>
        void ImportZeroSim(ZOSimDocumentRoot documentRoot, JObject json);

        /// <summary>
        /// From the current state of the object it will generate the JSON for this object and any children.
        /// </summary>
        /// <param name="parent"></param>
        /// <returns></returns>
        JObject BuildJSON(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null);

        /// <summary>
        /// Sets the state of the object from JSON. JSON could come from file or network or wherever.
        /// </summary>
        /// <param name="json"></param>
        void LoadFromJSON(ZOSimDocumentRoot documentRoot, JObject json);
        
    }
}