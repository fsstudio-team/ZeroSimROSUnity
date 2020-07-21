using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.Util;


namespace ZO.ROS.Unity {
    public abstract class ZOROSUnityGameObjectBase : ZOGameObjectBase, ZOROSUnityInterface, ZOSerializationInterface {

        public string _ROSTopic = "";

        /// <summary>
        /// The ROS Topic.  For example "/zerosim/joint_states"
        /// </summary>
        /// <value></value>
        public virtual string ROSTopic {
            get => _ROSTopic;
            set => _ROSTopic = value;
        }

        /// <summary>
        /// The ROS Bridge singleton shortcut access.
        /// </summary>
        /// <value></value>
        protected ZOROSBridgeConnection ROSBridgeConnection {
            get {
                return ZOROSBridgeConnection.Instance;
            }
        }

        /// <summary>
        /// The ROS Unity Manger singleton shortcut access.
        /// </summary>
        /// <value></value>
        protected ZOROSUnityManager ROSUnityManager {
            get {
                return ZOROSUnityManager.Instance;
            }
        }


        public string _name;

        /// <summary>
        /// Unique name of the object.  For example: "joint.hinge_from_left_wheel"
        /// </summary>
        /// <value></value>
        public virtual string Name {
            get {
                if (string.IsNullOrEmpty(_name)) {
                    _name = Type + "_" + gameObject.name;
                }
                return _name;
            }

            set => _name = value;
        }

        // TODO: make a ZOReset in ZOGameObjectBase
        private void Reset() {
            // generate the default name
            string dummy = Name;
        }

        #region ZOSerializationInterface

        protected JObject _json;
        /// <summary>
        /// The ZOSim JSON
        /// </summary>
        /// <value></value>
        public virtual JObject JSON {
            get => _json;
            set => _json = value;
        }

        /// <summary>
        /// The ZeroSim object type.  For example: "joint.hinge"
        /// </summary>
        /// <value></value>
        public virtual string Type {
            get { return "undefined"; }
        }

        /// <summary>
        /// OBSOLETE!!!!
        /// </summary>
        /// <param name="documentRoot"></param>
        /// <param name="json"></param>
        public virtual void ImportZeroSim(ZOSimDocumentRoot documentRoot, JObject json) {
            throw new System.NotImplementedException("Derived class should implement");
        }

        /// <summary>
        /// Dummy Serialize. Will throw exception if not implemented.`
        /// </summary>
        /// <param name="documentRoot"></param>
        /// <param name="parent"></param>
        /// <returns></returns>
        public virtual JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            throw new System.NotImplementedException("Derived class should implement");
            return null;
        }

        /// <summary>
        /// Dummy deserialize.  Will throw exception if not implemented.
        /// </summary>
        /// <param name="documentRoot"></param>
        /// <param name="json"></param>
        public virtual void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            throw new System.NotImplementedException("Derived class should implement");
        }


        /// <summary>
        /// The ZOSim document root
        /// </summary>
        /// <value></value>
        public ZOSimDocumentRoot DocumentRoot {
            get {
                // first check if we are the document root
                ZOSimDocumentRoot docRoot = this.GetComponent<ZOSimDocumentRoot>();
                if (docRoot == null) {
                    // search the parents
                    docRoot = this.GetComponentInParent<ZOSimDocumentRoot>();
                }

                if (docRoot == null) {
                    Debug.LogWarning("WARNING: document root cannot be accessed!!!");
                    throw new System.Exception("WARNING: document root cannot be accessed!!!");
                }

                return docRoot;
            }
        }

        #endregion // ZOSerializationInterface

        #region ZOGameObjectBase

        /// <summary>
        /// On Unity Start will connect to ROS bridge connect and disconnect events.
        /// </summary>
        protected override void ZOStart() {
            // auto-connect to ROS Bridge connection and disconnect events
            ZOROSUnityManager.Instance.ROSBridgeConnectEvent += OnROSBridgeConnected;
            ZOROSUnityManager.Instance.ROSBridgeDisconnectEvent += OnROSBridgeDisconnected;
        }

        /// <summary>
        /// On Unity Destroy will disconnect to ROS bridge connect and disconnect events.
        /// </summary>
        protected override void ZOOnDestroy() {
            ZOROSUnityManager.Instance.ROSBridgeConnectEvent -= OnROSBridgeConnected;
            ZOROSUnityManager.Instance.ROSBridgeDisconnectEvent -= OnROSBridgeDisconnected;
        }

        #endregion // ZOGameObjectBase
        
        public abstract void OnROSBridgeConnected(object rosUnityManager);

        public abstract void OnROSBridgeDisconnected(object rosUnityManager);


    }
}