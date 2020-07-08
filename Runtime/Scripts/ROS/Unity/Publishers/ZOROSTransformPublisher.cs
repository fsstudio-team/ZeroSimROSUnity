using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ZO.Util;
using ZO.ROS.MessageTypes.Geometry;

namespace ZO.ROS.Unity.Publisher {
    public class ZOROSTransformPublisher : ZOROSUnityGameObjectBase {
        public string _frameId = "";
        public string _childFrameId = "";
        public string ChildFrameID {
            get => _childFrameId;
            set => _childFrameId = value;
        }
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
    }
}