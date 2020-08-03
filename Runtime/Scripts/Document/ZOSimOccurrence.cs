using System.Linq;
using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEditor;
using Newtonsoft.Json.Linq;
using ZO.Physics;
using ZO.Util.Extensions;
using ZO.ROS.Controllers;
using ZO.Sensors;
using ZO.ROS.Unity;
using ZO.ROS.Unity.Publisher;

namespace ZO {

    /// <summary>
    /// A `ZOSimOccurence` is similar to a Unity GameObject but contains additional "meta" info specific
    /// to ZoSim.  It is responsible for serialization/deserialization of ZoSim JSON.  
    /// Every Unity GameObject that needs to serialize or interact with Zero Sim should
    /// have a `ZOSimOccurrence` as a component. 
    /// </summary>
    public class ZOSimOccurrence : MonoBehaviour, ZOSerializationInterface {

        [ZO.Util.ZOReadOnly] [SerializeField] public ZOSimDocumentRoot _documentRoot;


        /// <summary>
        /// Every ZoSim configuration has a `ZOSimDocumentRoot` as the root component.
        /// This property returns the ZoSim document root of this configuration.
        /// </summary>
        /// <value></value>
        public ZOSimDocumentRoot DocumentRoot {
            set { _documentRoot = value; }
            get {
                if (_documentRoot == null) {  // traverse up hierarchy to find parent base component
                    Transform parent = transform.parent;
                    while (parent != null) {
                        _documentRoot = parent.GetComponent<ZOSimDocumentRoot>();
                        if (_documentRoot != null) {
                            break;  // found
                        }
                        parent = parent.transform.parent; // keep traversing up
                    }
                }
                
                Assert.IsNotNull(_documentRoot, "ERROR: a ZOSimOccurrence needs a ZOSimDocumentRoot at the root of the hierarchy.");

                return _documentRoot;
            }
        }

        private void Start() {
            if (Application.IsPlaying(gameObject) == false) { // In Editor Mode 
                // update root component
                ZOSimDocumentRoot rootComponent = DocumentRoot;
            }
        }

        /// <summary>
        /// Finds an occurrence in this configuragtion 
        /// </summary>
        /// <param name="occurrenceName"></param>
        /// <returns></returns>
        public ZOSimOccurrence GetOccurrence(string occurrenceName) {
            Transform t = transform.Find(occurrenceName);
            if (t != null) {
                ZOSimOccurrence simOccurrence = t.GetComponent<ZOSimOccurrence>();
                return simOccurrence;
            }
            return null;
        }


        /// <summary>
        /// Finds a hinge joint by name.
        /// </summary>
        /// <param name="name"></param>
        /// <returns></returns>
        public ZOHingeJoint GetHingeJointNamed(string name) {
            ZOHingeJoint[] hingeJoints = GetComponents<ZOHingeJoint>();
            foreach (ZOHingeJoint hingeJoint in hingeJoints) {
                if (hingeJoint.Name == name) {
                    return hingeJoint;
                }
            }
            return null;
        }

        #region ZOSerializationInterface
        private JObject _json;
        public JObject JSON {
            get => _json;
            set => _json = value;
        }

        public string Name {
            get {
                return gameObject.name;
            }
            private set => gameObject.name = value;
        }

        public string Type {
            get {
                return "occurrence";
            }
        }


        public void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            DocumentRoot = documentRoot;
            Name = json["name"].Value<string>();
            JSON = json;

            // get the transform
            Quaternion rotation = Quaternion.identity;
            Vector3 translation = Vector3.zero;
            Vector3 scale = Vector3.one;
            if (json.ContainsKey("transform")) {  // has 4x4 matrix for transform
                // set the occurrence GameObject transform
                List<float> transformList = json["transform"].ToObject<List<float>>();
                Vector4 c0 = new Vector4(transformList[0], transformList[1], transformList[2], transformList[3]);
                Vector4 c1 = new Vector4(transformList[4], transformList[5], transformList[6], transformList[7]);
                Vector4 c2 = new Vector4(transformList[8], transformList[9], transformList[10], transformList[11]);
                Vector4 c3 = new Vector4(transformList[12], transformList[13], transformList[14], transformList[15]);
                Matrix4x4 matrix4X4 = new Matrix4x4(c0, c1, c2, c3);
                matrix4X4 = matrix4X4.transpose;  // go from fusion360 row major to Unity column major
                rotation = ZO.Math.ZOMatrix4x4Util.GetRotation(matrix4X4);
                translation = ZO.Math.ZOMatrix4x4Util.GetTranslation(matrix4X4);
                scale = ZO.Math.ZOMatrix4x4Util.GetScale(matrix4X4);
            }

            translation = json.ToVector3OrDefault("translation", translation);
            rotation = json.ToQuaternionOrDefault("rotation_quaternion", rotation);
            scale = json.ToVector3OrDefault("scale", scale);
            rotation = Quaternion.Euler(json.ToVector3OrDefault("rotation_euler_degrees", rotation.eulerAngles));

            this.transform.localRotation = rotation;
            this.transform.localScale = scale;
            this.transform.localPosition = translation;

            // load primitive type if exists
            if (json.ContainsKey("primitive")) {
                JObject primitiveJSON = json["primitive"].Value<JObject>();
                DeserializeGeometricPrimitive(this.gameObject, primitiveJSON);
            }


            // load rigid body if exists
            if (json.ContainsKey("rigidbody")) {
                JObject rbJson = json["rigidbody"].Value<JObject>();
                Rigidbody rb = GetComponent<Rigidbody>();

                if (rb == null) {
                    rb = this.gameObject.AddComponent<Rigidbody>();
                }
                rb.mass = rbJson.ContainsKey("mass") ? rbJson["mass"].Value<float>() : 1.0f;
                rb.drag = rbJson.ContainsKey("drag") ? rbJson["drag"].Value<float>() : 0;
                rb.angularDrag = rbJson.ContainsKey("angular_drag") ? rbJson["angular_drag"].Value<float>() : 0.01f;
                rb.useGravity = rbJson.ContainsKey("use_gravity") ? rbJson["use_gravity"].Value<bool>() : true;
                rb.isKinematic = rbJson.ContainsKey("is_kinematic") ? rbJson["is_kinematic"].Value<bool>() : false;
            }


            // load the joints
            // TODO: change this to be some sort of factory 
            if (json.ContainsKey("joints")) {
                foreach (JObject jointJSON in json["joints"].Value<JArray>()) {
                    if (jointJSON["type"].Value<string>() == "joint.hinge") {
                        ZOHingeJoint hingeJoint = null;
                        // check if joint exists 
                        ZOHingeJoint[] existingHingeJoints = this.GetComponents<ZOHingeJoint>();
                        foreach (ZOHingeJoint hj in existingHingeJoints) {
                            if (hj.Name == jointJSON["name"].Value<string>()) {
                                hingeJoint = hj;
                            }
                        }
                        // if hinge joint doesn't exist then create it
                        if (hingeJoint == null) {
                            hingeJoint = this.gameObject.AddComponent<ZOHingeJoint>();
                            hingeJoint.CreateRequirements();
                        }

                        hingeJoint.Deserialize(documentRoot, jointJSON);
                    }

                    // handle articulated body joints
                    if (jointJSON["type"].Value<string>().Contains("joint.articulated_body")) {
                        ZOArticulatedBody articulatedBody = this.GetComponent<ZOArticulatedBody>(); // remember that there can only ever be one articulated body joint per gameobject
                        if (articulatedBody == null) {
                            articulatedBody = this.gameObject.AddComponent<ZOArticulatedBody>();
                            articulatedBody.CreateRequirements();
                        }
                        articulatedBody.Deserialize(documentRoot, jointJSON);
                    }
                }
            }

            // load the controllers
            // TODO: change this to be some sort of factory to allow easy third party extension
            if (json.ContainsKey("controllers")) {
                foreach (JObject controllerJSON in json["controllers"].Value<JArray>()) {
                    if (controllerJSON["type"].Value<string>() == "controller.differential_drive") {
                        ZODifferentialDriveController controller = null;
                        // Type t = System.Type.GetType("blah");
                        // this.gameObject.AddComponent(t);
                        // check if controller exists 
                        ZODifferentialDriveController[] existingControllers = this.GetComponents<ZODifferentialDriveController>();
                        foreach (ZODifferentialDriveController cont in existingControllers) {
                            if (cont.Name == controllerJSON["name"].Value<string>()) {
                                controller = cont;
                                break;
                            }
                        }
                        // if controller doesn't exist then create it
                        if (controller == null) {
                            controller = this.gameObject.AddComponent<ZODifferentialDriveController>();
                        }

                        // finally load from JSON
                        controller.Deserialize(documentRoot, controllerJSON);
                    }
                }
            }

            // load ros specific stuff
            // TODO: change this to be some sort of factory
            if (json.ContainsKey("ros")) {
                foreach (JObject rosJSON in json["ros"].Value<JArray>()) {
                    if (rosJSON["type"].Value<string>() == "ros.publisher.transform") {
                        ZOROSTransformPublisher transformPublisher = null;
                        // check if transform publisher exists
                        ZOROSTransformPublisher[] transformPublishers = this.GetComponents<ZOROSTransformPublisher>();
                        foreach (ZOROSTransformPublisher tf in transformPublishers) {
                            if (tf.Name == rosJSON["name"].Value<string>()) {
                                transformPublisher = tf;
                                break;
                            }
                        }

                        // if transform publisher doesn't exist create it
                        if (transformPublisher == null) {
                            transformPublisher = this.gameObject.AddComponent<ZOROSTransformPublisher>();
                        }
                        transformPublisher.Deserialize(documentRoot, rosJSON);
                    }

                    if (rosJSON["type"].Value<string>() == "ros.publisher.image") {
                        ZOROSImagePublisher publisher = null;
                        // check if transform publisher exists
                        ZOROSImagePublisher[] publishers = this.GetComponents<ZOROSImagePublisher>();
                        foreach (ZOROSImagePublisher p in publishers) {
                            if (p.Name == rosJSON["name"].Value<string>()) {
                                publisher = p;
                                break;
                            }
                        }

                        // if publisher doesn't exist create it
                        if (publisher == null) {
                            publisher = this.gameObject.AddComponent<ZOROSImagePublisher>();
                        }
                        publisher.Deserialize(documentRoot, rosJSON);
                    }

                    if (rosJSON["type"].Value<string>() == "ros.publisher.scan") {
                        ZOROSLaserScanPublisher publisher = null;
                        // check if transform publisher exists
                        ZOROSLaserScanPublisher[] publishers = this.GetComponents<ZOROSLaserScanPublisher>();
                        foreach (ZOROSLaserScanPublisher p in publishers) {
                            if (p.Name == rosJSON["name"].Value<string>()) {
                                publisher = p;
                                break;
                            }
                        }

                        // if publisher doesn't exist create it
                        if (publisher == null) {
                            publisher = this.gameObject.AddComponent<ZOROSLaserScanPublisher>();
                        }
                        publisher.Deserialize(documentRoot, rosJSON);
                    }


                }
            }

            // load sensors
            // TODO: change this to be some sort of factory
            if (json.ContainsKey("sensors")) {
                foreach (JObject sensorJSON in json["sensors"].Value<JArray>()) {
                    if (sensorJSON["type"].Value<string>() == "sensor.rgbcamera") {
                        ZORGBCamera rgbCamera = null;
                        // check if rgb camera sensor exists
                        ZORGBCamera[] cameras = this.GetComponents<ZORGBCamera>();
                        foreach (ZORGBCamera camera in cameras) {
                            if (camera.Name == sensorJSON["name"].Value<string>()) {
                                rgbCamera = camera;
                                break;
                            }
                        }
                        // if it doesn't exist then create it
                        if (rgbCamera == null) {
                            rgbCamera = this.gameObject.AddComponent<ZORGBCamera>();
                        }
                        rgbCamera.Deserialize(documentRoot, sensorJSON);
                    }

                    if (sensorJSON["type"].Value<string>() == "sensor.lidar2d") {
                        ZOLIDAR2D lidar2d = null;
                        // check if rgb camera sensor exists
                        ZOLIDAR2D[] cameras = this.GetComponents<ZOLIDAR2D>();
                        foreach (ZOLIDAR2D lid2d in cameras) {
                            if (lid2d.Name == sensorJSON["name"].Value<string>()) {
                                lidar2d = lid2d;
                                break;
                            }
                        }
                        // if it doesn't exist then create it
                        if (lidar2d == null) {
                            lidar2d = this.gameObject.AddComponent<ZOLIDAR2D>();
                        }
                        lidar2d.Deserialize(documentRoot, sensorJSON);
                    }

                }
            }

            if (json.ContainsKey("visuals")) {

                // create a "visuals" gameobject "container"
                GameObject visualsContainerGo = new GameObject("visuals");
                visualsContainerGo.transform.parent = this.transform;
                visualsContainerGo.transform.localPosition = Vector3.zero;
                visualsContainerGo.transform.localRotation = Quaternion.identity;

                foreach (JObject visualJSON in json["visuals"].Value<JArray>()) {

                    if (visualJSON["type"].Value<string>() == "primitive.mesh") {
                        UnityEngine.Object[] assetObjects = DocumentRoot.FindAssetsByName(visualJSON["name"].Value<string>());
                        if (assetObjects.Length > 0) {
#if UNITY_EDITOR
                            GameObject visualGo = PrefabUtility.InstantiatePrefab(assetObjects[0]) as GameObject; // BUGBUG: only gets the first object in assetObjects

#else // runtime
                            GameObject visualGo = Instantiate(assetObjects[0] as GameObject); // BUGBUG: only gets the first object
#endif                            
                            visualGo.name = visualJSON["name"].Value<string>();
                            visualGo.transform.parent = visualsContainerGo.transform;

                            visualGo.transform.localPosition = visualJSON.ToVector3OrDefault("translation", visualGo.transform.localPosition);
                            visualGo.transform.localRotation = visualJSON.ToQuaternionOrDefault("rotation_quaternion", visualGo.transform.localRotation);
                            visualGo.transform.localRotation = Quaternion.Euler(visualJSON.ToVector3OrDefault("rotation_euler_degrees", visualGo.transform.localRotation.eulerAngles));
                            visualGo.transform.localScale = visualJSON.ToVector3OrDefault("scale", visualGo.transform.localScale);



                        } else {
                            Debug.LogWarning("WARNING: Could not load asset: " + visualJSON["name"].Value<string>() + " may need to add to the asset bundle");

                        }
                    } else {
                        // Handle geometric primitive e.g., Box, Capsule, Sphere, Cylinder
                        // create a visual gameobject
                        GameObject visualGo = new GameObject(visualJSON["name"].Value<string>());

                        // parent the visuals container
                        visualGo.transform.parent = visualsContainerGo.transform;

                        DeserializeGeometricPrimitive(visualGo, visualJSON);

                    }

                }
            }

            if (json.ContainsKey("collisions")) {
                // create a "visuals" gameobject "container"
                GameObject collisionsContainerGo = new GameObject("collisions");
                collisionsContainerGo.transform.parent = this.transform;
                collisionsContainerGo.transform.localPosition = Vector3.zero;
                collisionsContainerGo.transform.localRotation = Quaternion.identity;

                foreach (JObject collisionJSON in json["collisions"].Value<JArray>()) {

                    if (collisionJSON["type"].Value<string>() == "primitive.mesh") {
                        UnityEngine.Object[] assetObjects = DocumentRoot.FindAssetsByName(collisionJSON["name"].Value<string>());
                        if (assetObjects.Length > 0) {
#if UNITY_EDITOR
                            GameObject collisionGo = PrefabUtility.InstantiatePrefab(assetObjects[0]) as GameObject; // BUGBUG: only gets the first object in assetObjects

#else // runtime

                            GameObject collisionGo = Instantiate(assetObjects[0] as GameObject); // BUGBUG: only gets the first object
#endif                            
                            collisionGo.name = collisionJSON["name"].Value<string>();
                            collisionGo.transform.parent = collisionsContainerGo.transform;

                            collisionGo.transform.localPosition = collisionJSON.ToVector3OrDefault("translation", collisionGo.transform.localPosition);
                            collisionGo.transform.localRotation = collisionJSON.ToQuaternionOrDefault("rotation_quaternion", collisionGo.transform.localRotation);
                            collisionGo.transform.localRotation = Quaternion.Euler(collisionJSON.ToVector3OrDefault("rotation_euler_degrees", collisionGo.transform.localRotation.eulerAngles));
                            collisionGo.transform.localScale = collisionJSON.ToVector3OrDefault("scale", collisionGo.transform.localScale);

                            MeshCollider meshCollider = collisionGo.AddComponent<MeshCollider>();
                            // meshCollider.sharedMesh = collisionGo.GetComponentInChildren<Mesh>();
                            meshCollider.convex = collisionJSON.ValueOrDefault<bool>("is_convex", meshCollider.convex);

                            if (collisionJSON.ContainsKey("physics_material")) {
                                JObject physicsMaterial = collisionJSON["physics_material"].Value<JObject>();
                                meshCollider.sharedMaterial.bounciness = physicsMaterial.ValueOrDefault("bounciness", meshCollider.sharedMaterial.bounciness);
                                meshCollider.sharedMaterial.dynamicFriction = physicsMaterial.ValueOrDefault("dynamic_friction", meshCollider.sharedMaterial.dynamicFriction);
                                meshCollider.sharedMaterial.staticFriction = physicsMaterial.ValueOrDefault("static_friction", meshCollider.sharedMaterial.staticFriction);

                            }

                        } else {
                            Debug.LogWarning("WARNING: Could not load asset: " + collisionJSON["name"].Value<string>() + " may need to add to the asset bundle");

                        }
                    } else {
                        // Handle geometric primitive e.g., Box, Capsule, Sphere, Cylinder
                        // create a visual gameobject
                        GameObject collisionGo = new GameObject(collisionJSON["name"].Value<string>());

                        // parent the visuals container
                        collisionGo.transform.parent = collisionsContainerGo.transform;

                        DeserializeGeometricPrimitive(collisionGo, collisionJSON);


                    }

                }
            }


            // load component if exists (this is usually from the CAD import) and not necessarily common if we created ZoSim exclusively in Unity
            if (json.ContainsKey("component_name") == true) {
                JObject componentJson = DocumentRoot.GetComponentJSON(json["component_name"].Value<string>());

                if (componentJson.ContainsKey("visual_mesh_file") == true) {
                    // create a visual gameobject
                    GameObject visualsGo = new GameObject("visuals");
                    visualsGo.transform.parent = gameObject.transform;
                    visualsGo.transform.localPosition = Vector3.zero;
                    visualsGo.transform.localRotation = Quaternion.identity;

                    // Find the visual mesh prefab associated with the occurrence component
                    string visual_mesh_file = componentJson["visual_mesh_file"].Value<string>();

#if UNITY_EDITOR // cannot deal with prefabs during runtime.  

                    // we are doing an initial import so what we want to do is apply any mesh scaling
                    // TODO:  we should be doing this in ZOImportZeroSim.cs not here
                    Debug.Log("INFO: visual_obj_mesh_file: " + visual_mesh_file);
                    string visualMeshFile = Path.GetFileNameWithoutExtension(componentJson["visual_mesh_file"].Value<string>());
                    Debug.Log("INFO: visual mesh file path: " + visualMeshFile);
                    string[] visualMeshPrefabGUIDs = AssetDatabase.FindAssets(visualMeshFile);

                    // NOTE: should only ever be one prefab but we will just go ahead and go through all potential returns   
                    foreach (string meshPrefabGUID in visualMeshPrefabGUIDs) {
                        // string meshPrefabGUID = visualMeshPrefabGUIDs[0];  //BUGBUG:  Always first one found
                        string visualMeshPrefabPath = AssetDatabase.GUIDToAssetPath(meshPrefabGUID);

                        if (visualMeshPrefabPath.Contains("/" + visualMeshFile + ".obj") || visualMeshPrefabPath.Contains("/" + visualMeshFile + ".fbx")) {
                            Debug.Log("INFO: Found visual mesh prefab: " + visualMeshPrefabPath);

                            GameObject visualMeshGoPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(visualMeshPrefabPath);
                            GameObject visualMeshGo = PrefabUtility.InstantiatePrefab(visualMeshGoPrefab, visualsGo.transform) as GameObject;

                            Vector3 newScale = visualMeshGo.transform.localScale;
                            List<float> meshTransformScale = DocumentRoot.JSON["mesh_transform_scale"].ToObject<List<float>>();

                            newScale.x = newScale.x * meshTransformScale[0];
                            newScale.y = newScale.y * meshTransformScale[1];
                            newScale.z = newScale.z * meshTransformScale[2];
                            visualMeshGo.transform.localScale = newScale;

                        }
                    }
#else // NOT UNITY_EDITOR
                    // Load from asset bundle
                    GameObject meshGo = DocumentRoot.AssetBundle.LoadAsset<GameObject>(visual_mesh_file);
                    if (meshGo != null) {
                        Vector3 position = Vector3.zero; // TODO: store position
                        Quaternion r = Quaternion.identity; // TODO: store rotation
                        GameObject instance = Instantiate(meshGo, position, r);
                        instance.name = visual_mesh_file;//visualJSON["model_name"].Value<string>();
                        instance.transform.parent = visualsGo.transform;
                        instance.transform.localPosition = position;
                        instance.transform.localRotation = rotation;

                    } else { // error loading asset
                        Debug.LogWarning("WARNING: ZOSimOccurrence::LoadFromJSON could not load model: " + visual_mesh_file
                        + " Make sure the model is in the AssetBundle");
                    }

#endif
                }

                if (componentJson.ContainsKey("collision_meshes")) {
                    // create a collision gameobject
                    GameObject collisionsGo = new GameObject("collisions");
                    collisionsGo.transform.parent = gameObject.transform;
                    collisionsGo.transform.localPosition = Vector3.zero;
                    collisionsGo.transform.localRotation = Quaternion.identity;

                    // find the collision mesh prefabs associated with the occurrence component
                    foreach (string fileName in componentJson["collision_meshes"]) {
                        string collisionMeshFile = Path.GetFileNameWithoutExtension(fileName);
#if UNITY_EDITOR   
                        string[] collisionMeshPrefabGUIDS = AssetDatabase.FindAssets(collisionMeshFile);

                        foreach (string collisionMeshPrefabGUID in collisionMeshPrefabGUIDS) {
                            string collisionMeshPrefabPath = AssetDatabase.GUIDToAssetPath(collisionMeshPrefabGUID);
                            string collisionMeshPrefabPathBaseName = Path.GetFileNameWithoutExtension(collisionMeshPrefabPath);
                            // FindAssets will glob a bunch of assets that aren't exactly the one we want so make sure here
                            if (collisionMeshPrefabPathBaseName.Equals(collisionMeshFile)) {
                                Debug.Log("INFO: Found collision mesh prefab: " + collisionMeshPrefabPath);
                                GameObject collisionMeshGoPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(collisionMeshPrefabPath);

                                // apply transforms on collision mesh
                                GameObject collisionMeshGo = new GameObject(collisionMeshFile);
                                collisionMeshGo.transform.parent = collisionsGo.transform;
                                collisionMeshGo.transform.localPosition = Vector3.zero;
                                collisionMeshGo.transform.localRotation = Quaternion.identity;
                                Vector3 newScale = collisionMeshGo.transform.localScale;
                                List<float> meshTransformScale = DocumentRoot.JSON["mesh_transform_scale"].ToObject<List<float>>();
                                newScale.x = newScale.x * meshTransformScale[0];
                                newScale.y = newScale.y * meshTransformScale[1];
                                newScale.z = newScale.z * meshTransformScale[2];
                                collisionMeshGo.transform.localScale = newScale;

                                MeshCollider meshCollider = collisionMeshGo.AddComponent<MeshCollider>();
                                MeshFilter meshFilter = collisionMeshGoPrefab.GetComponentInChildren<MeshFilter>();
                                meshCollider.sharedMesh = meshFilter.sharedMesh;
                                meshCollider.convex = true;

                            }

                        }
#else // NOT UNITY EDITOR
                        // TODO: handle collision meshes that are in the asset bundle
#endif
                    }

                }

            }

            if (json.ContainsKey("children")) {
                // go through children
                foreach (JObject occurrenceJSON in JSON["children"].Value<JArray>()) {
                    string occurrenceName = occurrenceJSON["name"].Value<string>();

                    ZOSimOccurrence simOccurrence = GetOccurrence(occurrenceName);
                    if (simOccurrence == null) {
                        // create new sim occurrence gameobject
                        GameObject go = new GameObject(occurrenceName);
                        go.transform.parent = this.transform;
                        simOccurrence = go.AddComponent<ZOSimOccurrence>();
                    }
                    simOccurrence.Deserialize(documentRoot, occurrenceJSON);
                }

            }

            //TODO: remove any children not in the children list

            //TODO:  apply any fixups such as joint connected bodies

        }


        /// <summary>
        /// Build the JSON representation.
        /// TODO:  This should only really be done in Editor mode. Otherwise it should just be the immutable JSON from a file (or network)
        /// </summary>
        /// <param name="documentRoot">ZOSimDocumentRoot</param>
        /// <param name="parent">parent, null if none</param>
        /// <returns></returns>
        public JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject occurrence = new JObject();

            // TODO: check with document root if there is a zosim component reference 
            occurrence["name"] = Name;

            occurrence["translation"] = transform.localPosition.ToJSON();
            occurrence["rotation_quaternion"] = transform.localRotation.ToJSON();
            occurrence["scale"] = transform.localScale.ToJSON();

            if (parent != null) {
                occurrence["parent_name"] = ((ZOSimOccurrence)parent).Name;
            }

            // build 3d primitive (if directly attached to object)
            JObject primitiveJSON = SerializeGeometricPrimitive(this.gameObject);
            if (primitiveJSON != null) {
                occurrence["primitive"] = primitiveJSON;
            }


            // rigid body
            Rigidbody rigidbody = GetComponent<Rigidbody>();
            if (rigidbody != null) {
                occurrence["rigidbody"] = new JObject(
                    new JProperty("mass", rigidbody.mass),
                    new JProperty("drag", rigidbody.drag),
                    new JProperty("angular_drag", rigidbody.angularDrag),
                    new JProperty("use_gravity", rigidbody.useGravity),
                    new JProperty("is_kinematic", rigidbody.isKinematic)
                );
            } else if (occurrence.ContainsKey("rigidbody") == true) {
                occurrence.Remove("rigidbody");
            }

            // Joints & Controllers
            // TODO: throw all of this into a single "sub_components" array
            JArray joints = new JArray();
            JArray controllers = new JArray();
            JArray ros = new JArray();
            JArray sensors = new JArray();

            // FixedJoint
            // TODO: make a ZOFixedJoint 
            foreach (FixedJoint fixedJoint in GetComponents<FixedJoint>()) {
                JObject fixedJointJSON = new JObject(
                    new JProperty("type", "fixed")
                );
                if (fixedJoint.connectedBody != null) {
                    fixedJointJSON["connected_occurrence"] = fixedJoint.connectedBody.gameObject.name;
                }

                joints.Add(fixedJointJSON);

            }

            // serialize the rest of objects that implement the ZOSerializationInterface
            ZOSerializationInterface[] simTypes = GetComponents<ZOSerializationInterface>();
            foreach (ZOSerializationInterface simType in simTypes) {
                string[] subtypes = simType.Type.Split('.');

                // handle joint types
                if (subtypes[0] == "joint") {
                    JObject jointJSON = simType.Serialize(documentRoot, this);
                    joints.Add(jointJSON);
                } else if (subtypes[0] == "controller") {
                    JObject contollerJSON = simType.Serialize(documentRoot, this);
                    controllers.Add(contollerJSON);
                } else if (subtypes[0] == "ros") {
                    JObject rosJSON = simType.Serialize(documentRoot, this);
                    ros.Add(rosJSON);
                } else if (subtypes[0] == "sensor") {
                    JObject rosJSON = simType.Serialize(documentRoot, this);
                    sensors.Add(rosJSON);
                } else if (subtypes[0] == "occurrence") {
                    // DO NOTHING
                } else {
                    Debug.LogWarning("WARNING: Do not understand how to serialize the JSON of type: " + simType.Type);
                }

            }


            if (joints.Count > 0) {
                occurrence["joints"] = joints;
            }
            if (controllers.Count > 0) {
                occurrence["controllers"] = controllers;
            }
            if (ros.Count > 0) {
                occurrence["ros"] = ros;
            }
            if (sensors.Count > 0) {
                occurrence["sensors"] = sensors;
            }

            // go through the children
            JArray children = new JArray();
            JArray visuals = new JArray();
            JArray collisions = new JArray();
            foreach (Transform child in transform) {

                // BUGBUG: visual models can only be determined at Unity Editor time not runtime... hmmm...

                // check for any visuals
                // NOTE: a gameobject named visuals is treated as a special container of visual objects
                if (child.name == "visuals") {
                    // go through the children of the visuals and get all the models
                    foreach (Transform visualsChild in child) {
                        // check if it is a primitive type (cube, sphere, cylinder, etc)
                        primitiveJSON = SerializeGeometricPrimitive(visualsChild.gameObject);
                        if (primitiveJSON != null) {
                            visuals.Add(primitiveJSON);
                        } else {
                            // not a geometric primitive type but likely a mesh so lets try to find the mesh in the AssetBundle
                            UnityEngine.Object[] assetObjects = DocumentRoot.FindAssetsByName(visualsChild.name);
                            if (assetObjects.Length > 0) {
                                JObject meshPrimitiveJSON = new JObject(
                                    new JProperty("type", "primitive.mesh"),
                                    new JProperty("name", visualsChild.name),
                                    new JProperty("has_collisions", false), // BUGBUG: in theory we could have collisions but we hardwire to not
                                    new JProperty("translation", visualsChild.localPosition.ToJSON()),
                                    new JProperty("rotation_quaternion", visualsChild.localRotation.ToJSON()),
                                    new JProperty("scale", visualsChild.localScale.ToJSON())
                                );

                                visuals.Add(meshPrimitiveJSON);
                            } else {
                                Debug.LogWarning("WARNING: asset: " + visualsChild.name + "does not exist in bundle: " + DocumentRoot.AssetBundle.name + " try to add the asset to the asset bundle");
                            }
                            // Debug.Log("INFO: found mesh in asset bundle: " + visualsChild.name);
                        }

#if UNITY_EDITOR // cannot deal with prefabs during runtime.  FIXME?
                        // PrefabAssetType prefabAssetType = PrefabUtility.GetPrefabAssetType(visualsChild);
                        // Debug.Log("INFO: visuals prefab asset type: " + prefabAssetType.ToString());
                        // if (prefabAssetType == PrefabAssetType.Model) {
                        //     JObject modelJSON = new JObject(
                        //         new JProperty("model_name", visualsChild.name)
                        //     );
                        //     visuals.Add(modelJSON);
                        // }
#endif // #if UNITY_EDITOR
                    }
                }

                // check for any visuals
                // NOTE: a gameobject named collisions is treated as a special container of collision objects
                if (child.name == "collisions") {
                    // go through the children of the visuals and get all the models
                    foreach (Transform collisionChild in child) {
                        // check if it is a primitive type (cube, sphere, cylinder, etc)
                        primitiveJSON = SerializeGeometricPrimitive(collisionChild.gameObject);
                        if (primitiveJSON != null) {
                            collisions.Add(primitiveJSON);
                        } else {
                            // not a geometric primitive type but likely a mesh so lets try to find the mesh in the AssetBundle
                            // GameObject meshGo = DocumentRoot.AssetBundle.LoadAsset<GameObject>(collisionChild.name);
                            UnityEngine.Object[] assetObjects = DocumentRoot.FindAssetsByName(collisionChild.name);

                            if (assetObjects.Length > 0) {
                                MeshCollider collider = collisionChild.gameObject.GetComponent<MeshCollider>();
                                if (collider != null) {
                                    JObject meshPrimitiveJSON = new JObject(
                                        new JProperty("type", "primitive.mesh"),
                                        new JProperty("name", collisionChild.name),
                                        new JProperty("has_collisions", true),
                                        new JProperty("is_convex", collider.convex),

                                        new JProperty("translation", collisionChild.localPosition.ToJSON()),
                                        new JProperty("rotation_quaternion", collisionChild.localRotation.ToJSON()),
                                        new JProperty("scale", collisionChild.localScale.ToJSON())
                                    );

                                    if (collider.sharedMaterial != null) {
                                        primitiveJSON.Add("physics_material",
                                            new JObject(
                                                new JProperty("bounciness", collider.sharedMaterial.bounciness),
                                                new JProperty("dynamic_friction", collider.sharedMaterial.dynamicFriction),
                                                new JProperty("static_friction", collider.sharedMaterial.staticFriction)
                                            )
                                        );
                                    }

                                    collisions.Add(meshPrimitiveJSON);

                                } else {  // does not have a mesh collider attached
                                    Debug.LogWarning("WARNING: object in collisions does not have a proper MeshCollider: " + collisionChild.name);

                                }
                            }
                            // Debug.Log("INFO: found mesh in asset bundle: " + visualsChild.name);
                        }

                    }
                }

                // recursively go through all the children occurrences
                ZOSimOccurrence simOccurrence = child.GetComponent<ZOSimOccurrence>();
                if (simOccurrence) {
                    JObject child_json = simOccurrence.Serialize(documentRoot, this);
                    children.Add(child_json);
                }
            }

            if (visuals.Count > 0) {
                occurrence["visuals"] = visuals;
            }

            if (collisions.Count > 0) {
                occurrence["collisions"] = collisions;
            }

            if (children.Count > 0) {
                occurrence["children"] = children;
            }




            return occurrence;

        }

        /// <summary>
        /// Serialize a geometric primitive such as box, sphere, capsule and cylinder.
        /// NOTE:  Meshes are handled seperately.
        /// </summary>
        /// <param name="go"></param>
        /// <returns></returns>
        private static JObject SerializeGeometricPrimitive(GameObject go) {
            JObject primitiveJSON = null;
            // build 3d primitive if exists
            MeshFilter meshFilter = go.GetComponent<MeshFilter>();
            if (meshFilter) {

                MeshRenderer meshRenderer = go.GetComponent<MeshRenderer>();
                Collider collider = null;

                if (meshFilter.sharedMesh.name.Contains("Cube")) {
                    collider = go.GetComponent<BoxCollider>();
                    primitiveJSON = new JObject(
                        new JProperty("type", "primitive.cube"),
                        new JProperty("dimensions", go.transform.localScale.ToJSON()),
                        new JProperty("has_collisions", collider ? true : false),
                        new JProperty("color", meshRenderer.sharedMaterial.color.ToJSON())
                    );
                }
                if (meshFilter.sharedMesh.name.Contains("Sphere")) {
                    collider = go.GetComponent<SphereCollider>();
                    primitiveJSON = new JObject(
                        new JProperty("type", "primitive.sphere"),
                        new JProperty("dimensions", go.transform.localScale.ToJSON()),
                        new JProperty("has_collisions", collider ? true : false),
                        new JProperty("color", meshRenderer.sharedMaterial.color.ToJSON())
                    );
                }
                if (meshFilter.sharedMesh.name.Contains("Capsule")) {
                    collider = go.GetComponent<CapsuleCollider>();
                    primitiveJSON = new JObject(
                        new JProperty("type", "primitive.capsule"),
                        new JProperty("dimensions", go.transform.localScale.ToJSON()),
                        new JProperty("has_collisions", collider ? true : false),
                        new JProperty("color", meshRenderer.sharedMaterial.color.ToJSON())
                    );
                }
                if (meshFilter.sharedMesh.name.Contains("Cylinder")) {
                    collider = go.GetComponent<MeshCollider>();
                    primitiveJSON = new JObject(
                        new JProperty("type", "primitive.cylinder"),
                        new JProperty("dimensions", go.transform.localScale.ToJSON()),
                        new JProperty("has_collisions", collider ? true : false),
                        new JProperty("color", meshRenderer.sharedMaterial.color.ToJSON())
                    );
                }

                // add name
                primitiveJSON.Add("name", go.name);

                // add transform
                primitiveJSON.Add("translation", go.transform.localPosition.ToJSON());
                primitiveJSON.Add("rotation_quaternion", go.transform.localRotation.ToJSON());
                primitiveJSON.Add("scale", go.transform.localScale.ToJSON());


                // generate physics material if necessary (friction, restitution)
                if (collider != null && collider.sharedMaterial != null) {
                    primitiveJSON.Add("physics_material",
                        new JObject(
                            new JProperty("bounciness", collider.sharedMaterial.bounciness),
                            new JProperty("dynamic_friction", collider.sharedMaterial.dynamicFriction),
                            new JProperty("static_friction", collider.sharedMaterial.staticFriction)
                        )
                    );
                }

            }

            return primitiveJSON;

        }


        /// <summary>
        /// Deserialize a geometric primitive such as box, sphere, capsule and cylinder.
        /// NOTE:  Meshes are handled seperately.
        /// </summary>
        /// <param name="go"></param>
        /// <returns></returns>
        private static void DeserializeGeometricPrimitive(GameObject go, JObject primitiveJSON) {
            Collider collider = null;
            // get mesh filter. if it doesn't exist create it.
            MeshFilter meshFilter = go.GetComponent<MeshFilter>();
            MeshRenderer meshRenderer = null;
            if (meshFilter == null) {
                meshFilter = go.AddComponent<MeshFilter>();
                meshRenderer = go.AddComponent<MeshRenderer>();
                // meshRenderer.material = new Material(Shader.Find("Diffuse"));

                if (primitiveJSON["type"].Value<string>() == "primitive.cube") {
                    // meshFilter.sharedMesh = Resources.GetBuiltinResource<Mesh>("Cube.fbx");
                    GameObject tmp = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    meshFilter.sharedMesh = tmp.GetComponent<MeshFilter>().sharedMesh;
                    meshRenderer.sharedMaterial = tmp.GetComponent<MeshRenderer>().sharedMaterial;
                    GameObject.DestroyImmediate(tmp);

                    meshFilter.sharedMesh.name = "Cube Instance";
                    bool hasCollisions = primitiveJSON.ValueOrDefault<bool>("has_collisions", false);
                    if (hasCollisions) {
                        collider = go.AddComponent<BoxCollider>();
                    }
                }

                if (primitiveJSON["type"].Value<string>() == "primitive.sphere") {
                    GameObject tmp = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    meshFilter.sharedMesh = tmp.GetComponent<MeshFilter>().sharedMesh;
                    meshRenderer.sharedMaterial = tmp.GetComponent<MeshRenderer>().sharedMaterial;
                    GameObject.DestroyImmediate(tmp);

                    // meshFilter.sharedMesh = Resources.GetBuiltinResource<Mesh>("Sphere.fbx");
                    meshFilter.sharedMesh.name = "Sphere Instance";
                    bool hasCollisions = primitiveJSON.ValueOrDefault<bool>("has_collisions", false);
                    if (hasCollisions) {
                        collider = go.AddComponent<SphereCollider>();
                    }
                }

                if (primitiveJSON["type"].Value<string>() == "primitive.capsule") {
                    // meshFilter.sharedMesh = Resources.GetBuiltinResource<Mesh>("Capsule.fbx");
                    GameObject tmp = GameObject.CreatePrimitive(PrimitiveType.Capsule);
                    meshFilter.sharedMesh = tmp.GetComponent<MeshFilter>().sharedMesh;
                    meshRenderer.sharedMaterial = tmp.GetComponent<MeshRenderer>().sharedMaterial;
                    GameObject.DestroyImmediate(tmp);
                    meshFilter.sharedMesh.name = "Capsule Instance";
                    bool hasCollisions = primitiveJSON.ValueOrDefault<bool>("has_collisions", false);
                    if (hasCollisions) {
                        collider = go.AddComponent<CapsuleCollider>();
                    }
                }

                if (primitiveJSON["type"].Value<string>() == "primitive.cylinder") {
                    GameObject tmp = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                    meshFilter.sharedMesh = tmp.GetComponent<MeshFilter>().sharedMesh;
                    meshRenderer.sharedMaterial = tmp.GetComponent<MeshRenderer>().sharedMaterial;
                    GameObject.DestroyImmediate(tmp);
                    // meshFilter.sharedMesh = Resources.GetBuiltinResource<Mesh>("Cylinder.fbx");
                    meshFilter.sharedMesh.name = "Cylinder Instance";
                    bool hasCollisions = primitiveJSON.ValueOrDefault<bool>("has_collisions", false);
                    if (hasCollisions) {
                        MeshCollider meshCollider = go.AddComponent<MeshCollider>();
                        meshCollider.convex = true;
                        collider = meshCollider;
                    }
                }

                // set physics material if exists
                if (primitiveJSON.ContainsKey("physics_material")) {
                    JObject physicsMaterialJSON = primitiveJSON["physics_material"].Value<JObject>();
                    PhysicMaterial physicMaterial = new PhysicMaterial();
                    physicMaterial.bounciness = physicsMaterialJSON["bounciness"].Value<float>();
                    physicMaterial.dynamicFriction = physicsMaterialJSON["dynamic_friction"].Value<float>();
                    physicMaterial.staticFriction = physicsMaterialJSON["static_friction"].Value<float>();
                    collider.sharedMaterial = physicMaterial;
                }


                // set color
                meshRenderer = go.GetComponent<MeshRenderer>();
                meshRenderer.sharedMaterial.color = primitiveJSON.ToColorOrDefault("color", meshRenderer.sharedMaterial.color);

                // set the transform if exists
                go.transform.localPosition = primitiveJSON.ToVector3OrDefault("translation", go.transform.localPosition);
                go.transform.localRotation = primitiveJSON.ToQuaternionOrDefault("rotation_quaternion", go.transform.localRotation);
                go.transform.localRotation = Quaternion.Euler(primitiveJSON.ToVector3OrDefault("rotation_euler_degrees", go.transform.localRotation.eulerAngles));

                // scale is how Unity primitives are sized
                go.transform.localScale = primitiveJSON.ToVector3OrDefault("dimensions", go.transform.localScale);

            }

        }


        #endregion
    }
}