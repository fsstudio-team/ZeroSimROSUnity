using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Sensor;
using ZO.Physics;

namespace ZO.ROS.Unity.Publisher {
    public class ZOROSJointStatesPublisher : ZOROSUnityGameObjectBase {

        private JointStateMessage _jointStatesMessage = new JointStateMessage();

        protected override void ZOStart() {
            base.ZOStart();
            if (ZOROSBridgeConnection.Instance.IsConnected) {
                Initialize();
            }
        }

        protected override void ZOOnDestroy() {
            ROSBridgeConnection?.UnAdvertise(ROSTopic);
        }


        private void Initialize() {
            // advertise
            ROSBridgeConnection.Advertise(ROSTopic, _jointStatesMessage.MessageType);

        }


        protected override void ZOUpdateHzSynchronized() {
            _jointStatesMessage.header.Update();
            // _jointStatesMessage.header.frame_id = FrameID; TODO???

            // get all the joints
            // TODO: cache the joints assuming they don't change
            // TODO: we don't quite yet have a Joint base class fully implemented so we look for specific joints.
            ZOHingeJoint[] hingeJoints = this.GetComponentsInChildren<ZOHingeJoint>();

            // setup the message arrays
            _jointStatesMessage.name = new string[hingeJoints.Length];
            _jointStatesMessage.position = new double[hingeJoints.Length];
            _jointStatesMessage.velocity = new double[hingeJoints.Length];
            _jointStatesMessage.effort = new double[hingeJoints.Length];

            // fill in the arrays
            int i = 0;
            foreach (ZOHingeJoint hingeJoint in hingeJoints) {
                _jointStatesMessage.name[i] = hingeJoint.Name;
                _jointStatesMessage.position[i] = hingeJoint.AngleDegrees * Mathf.Deg2Rad;
                _jointStatesMessage.velocity[i] = hingeJoint.AngularVelocityDegrees * Mathf.Deg2Rad;
                _jointStatesMessage.effort[i] = hingeJoint.TorqueNewtonMeters;
                i++;
            }

            ROSBridgeConnection.Publish(_jointStatesMessage, ROSTopic, Name);
        }


        public override string Type {
            get { return "ros.publisher.joint_states"; }
        }

        public override JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("ros_topic", ROSTopic),
                new JProperty("update_rate_hz", UpdateRateHz)
            );
            JSON = json;
            return json;
        }

        public override void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            Name = json["name"].Value<string>();
            ROSTopic = json["ros_topic"].Value<string>();
            UpdateRateHz = json["update_rate_hz"].Value<float>();
        }

        public override void OnROSBridgeConnected(object rosUnityManager) {
            Debug.Log("INFO: ZOROSJointStatesPublisher::OnROSBridgeConnected");
            Initialize();

        }

        public override void OnROSBridgeDisconnected(object rosUnityManager) {
            Debug.Log("INFO: ZOROSJointStatesPublisher::OnROSBridgeDisconnected");
            ROSBridgeConnection?.UnAdvertise(ROSTopic);
        }


    }
}