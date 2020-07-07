using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ZO.Util;

namespace ZO.ROS.Unity {
    public abstract class ZOROSUnityGameObjectBase : ZOGameObjectBase, ZOROSUnityInterface {

        public string _ROSTopic = "";
        public string _ROSId = "";

        public virtual string ROSTopic {
            get => _ROSTopic;
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


        protected override void ZOStart() {
            // auto-connect to ROS Bridge connection and disconnect events
            ZOROSUnityManager.Instance.ROSBridgeConnectEvent.AddListener(OnROSBridgeConnected);
            ZOROSUnityManager.Instance.ROSBridgeDisconnectEvent.AddListener(OnROSBridgeConnected);
        }

        public virtual void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager, ZOROSBridgeConnection rosBridgeConnection) {
            Debug.Log("INFO: ZOROSUnityGameObjectBase::OnROSBridgeDisconnected");
        }

        public virtual void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager, ZOROSBridgeConnection rosBridgeConnection) {
            Debug.Log("INFO: ZOROSUnityGameObjectBase::OnROSBridgeDisconnected");
        }

    }
}