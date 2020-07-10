using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.Util;


namespace ZO.ROS.Unity {
    public abstract class ZOROSUnityGameObjectBase : ZOGameObjectBase, ZOROSUnityInterface, ZOSerializationInterface {

        public string _ROSTopic = "";
        public string _ROSId = "";

        public virtual string ROSTopic {
            get => ROSUnityManager.Namespace + "/" + _ROSTopic;
            set => _ROSTopic = value;
        }
        public virtual string ROSId {
            get => _ROSId;
            set => _ROSId = value;
        }

        protected ZOROSBridgeConnection ROSBridgeConnection {
            get {
                return ZOROSBridgeConnection.Instance;
            }
        }
        protected ZOROSUnityManager ROSUnityManager {
            get {
                return ZOROSUnityManager.Instance;
            }
        }

        protected JObject _json;
        public virtual JObject JSON {
            get => _json;
            set => _json = value;
        }

        public string _name;
        public virtual string Name {
            get {
                if (string.IsNullOrEmpty(_name)) {
                    _name = Type + "_" + gameObject.name;
                }
                return _name;
            }

            set => _name = value;
        }

        public virtual string Type {
            get { return "undefined"; }
        }

        public virtual void ImportZeroSim(ZOSimDocumentRoot documentRoot, JObject json) {
            throw new System.NotImplementedException("Derived class should implement");
        }

        public virtual JObject BuildJSON(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            throw new System.NotImplementedException("Derived class should implement");
            return null;
        }

        public virtual void LoadFromJSON(ZOSimDocumentRoot documentRoot, JObject json) {
            throw new System.NotImplementedException("Derived class should implement");
        }

        /// <summary>
        /// Will connect to ROS bridge connect and disconnect events.
        /// </summary>
        protected override void ZOStart() {
            // auto-connect to ROS Bridge connection and disconnect events
            ZOROSUnityManager.Instance.ROSBridgeConnectEvent += OnROSBridgeConnected;
            ZOROSUnityManager.Instance.ROSBridgeDisconnectEvent += OnROSBridgeDisconnected;
        }

        protected override void ZOOnDestroy() {
            ZOROSUnityManager.Instance.ROSBridgeConnectEvent -= OnROSBridgeConnected;
            ZOROSUnityManager.Instance.ROSBridgeDisconnectEvent -= OnROSBridgeDisconnected;
        }


        public abstract void OnROSBridgeConnected(object rosUnityManager);

        public abstract void OnROSBridgeDisconnected(object rosUnityManager);


    }
}