using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Geometry;
using ZO.ROS.Unity;

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
            if (ChildFrameID.Length == 0) {
                ChildFrameID = this.gameObject.name;
            }

            // TODO: autoset frameId by looking up the hierarch for a parent ZOROSTransformPublisher
            // if no parent found then set to "map"
        }
        protected override void ZOUpdateHzSynchronized() {
            _transformMessage.header.Update();
            _transformMessage.header.frame_id = FrameID;
            _transformMessage.child_frame_id = ChildFrameID;
            _transformMessage.UnityLocalTransform = this.transform;

            ROSUnityManager.BroadcastTransform(_transformMessage);

        }


        public override string Type {
            get { return "ros.publisher.transform"; }
        }

        public override JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("ros_topic", ROSTopic),
                new JProperty("update_rate_hz", UpdateRateHz),
                new JProperty("frame_id", FrameID),
                new JProperty("child_frame_id", ChildFrameID)
            );
            JSON = json;
            return json;
        }

        public override void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            Name = json["name"].Value<string>();
            FrameID = json["frame_id"].Value<string>();
            ROSTopic = json["ros_topic"].Value<string>();
            ChildFrameID = json["child_frame_id"].Value<string>();
            UpdateRateHz = json["update_rate_hz"].Value<float>();
        }

        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOROSTransformPublisher::OnROSBridgeConnected");

        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOROSTransformPublisher::OnROSBridgeDisconnected");
        }


    }
}