using System;
using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes;
using ZO.ROS.MessageTypes.Control;
using ZO.ROS.MessageTypes.Geometry;
using ZO.ROS.MessageTypes.Nav;
using ZO.ROS.MessageTypes.Trajectory;
using ZO.ROS;
using ZO.ROS.Unity;


namespace ZO.Controllers {
    public class ZOArmController : ZOROSUnityGameObjectBase {

        private ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal> _actionServer = new ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal>();
        private JointTrajectoryMessage _commandMessage = new JointTrajectoryMessage();

        protected override void ZOStart() {
            _actionServer.ROSTopic = "/arm_controller/follow_joint_trajectory";
            _actionServer.Name = "arm_controller";
            _actionServer.Initialize();
            base.ZOStart();
            if (ZOROSBridgeConnection.Instance.IsConnected) {
                Initialize();
            }
        }

        protected override void ZOOnDestroy() {
            _actionServer.Terminate();
            ROSBridgeConnection?.UnAdvertise(ROSTopic);
        }


        private void Initialize() {
            // advertise
            // ROSBridgeConnection.Advertise(ROSTopic, _jointStatesMessage.MessageType);

            // subscribe to the /arm_controller/command
            ROSBridgeConnection.Subscribe<JointTrajectoryMessage>("arm_controller", "/arm_controller/command", JointTrajectoryMessage.Type, OnControlMessageReceived);

        }

        public Task OnControlMessageReceived(ZOROSBridgeConnection rosBridgeConnection, ZOROSMessageInterface msg) {
            _commandMessage = (JointTrajectoryMessage)msg;
            Debug.Log("INFO:  Command message received.");
            // Debug.Log("INFO: Twist Message Received: linear " + _twistMessage.linear.ToString() + " angular: " + _twistMessage.angular.ToString());

            return Task.CompletedTask;
        }


        #region ZOSerializationInterface
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

        #endregion // ZOSerializationInterface

        #region ZOROSUnityInterface
        public override void OnROSBridgeConnected(object rosUnityManager) {
            Debug.Log("INFO: ZOArmController::OnROSBridgeConnected");
            Initialize();

        }

        public override void OnROSBridgeDisconnected(object rosUnityManager) {
            Debug.Log("INFO: ZOArmController::OnROSBridgeDisconnected");
            ROSBridgeConnection?.UnAdvertise(ROSTopic);
        }

        #endregion // ZOROSUnityInterface

    }
}