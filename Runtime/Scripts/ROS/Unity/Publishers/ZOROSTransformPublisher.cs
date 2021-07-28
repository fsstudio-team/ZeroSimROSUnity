using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Geometry;
using ZO.ROS.Unity;
using ZO.Document;

namespace ZO.ROS.Publisher {
    public class ZOROSTransformPublisher : ZOROSUnityGameObjectBase {
        public string _frameId = "";
        public string _childFrameId = "";

        /// <summary>
        /// The name of *THIS* frame.
        /// </summary>
        /// <value></value>
        public string ChildFrameID {
            get => _childFrameId;
            set => _childFrameId = value;
        }

        /// <summary>
        /// The name of the parent frame.
        /// </summary>
        /// <value></value>
        public string FrameID {
            get => _frameId;
            set => _frameId = value;
        }

        private TransformStampedMessage _transformMessage = new TransformStampedMessage();

        protected override void ZOStart() {
            base.ZOStart();
            // if the child frame id is not set then set it to be the name of this game object.
            if (string.IsNullOrEmpty(ChildFrameID) == true) {
                ChildFrameID = this.gameObject.name;
            }

        }
        protected override void ZOUpdateHzSynchronized() {
            _transformMessage.header.Update();
            _transformMessage.header.frame_id = FrameID;
            _transformMessage.child_frame_id = ChildFrameID;
            _transformMessage.FromLocalUnityTransformToROS(this.transform);

            ROSUnityManager.BroadcastTransform(_transformMessage);

        }

        protected override void ZOOnValidate() {
            base.ZOOnValidate();
            if (string.IsNullOrEmpty(ChildFrameID) == true) {
                ChildFrameID = Name;
            }
            if (string.IsNullOrEmpty(FrameID) == true) {
                if (transform.parent) {
                    FrameID = transform.parent.name;
                } else {
                    FrameID = ZOROSUnityManager.Instance.WorldFrameId;
                }
            }
            if (UpdateRateHz == 0) {
                UpdateRateHz = 10;
            }
        }

        public override string Type {
            get { return "ros.publisher.transform"; }
        }


        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOROSTransformPublisher::OnROSBridgeConnected");

        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOROSTransformPublisher::OnROSBridgeDisconnected");
        }


    }
}