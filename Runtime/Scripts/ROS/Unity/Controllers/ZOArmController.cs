using System;
using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes;
using ZO.ROS.MessageTypes.Control;
using ZO.ROS.MessageTypes.Trajectory;
using ZO.ROS.MessageTypes.ControllerManager;
using ZO.ROS;
using ZO.ROS.Unity;
using ZO.ROS.Unity.Service;


namespace ZO.ROS.Controllers {
    public class ZOArmController : ZOROSUnityGameObjectBase, ZOROSControllerInterface {

        private ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal> _actionServer = new ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal>();
        private JointTrajectoryMessage _commandMessage = new JointTrajectoryMessage();

        #region ZOGameObjectBase

        protected override void ZOAwake() {
            Name = "arm_controller";
        }
        protected override void ZOStart() {
            base.ZOStart();

            // register with controller manager
            ZOControllerManagerService.Instance.RegisterController(this);

            if (ZOROSBridgeConnection.Instance.IsConnected) {
                Initialize();
            }
        }

        protected override void ZOOnDestroy() {

            Terminate();
        }
        #endregion // ZOGameObjectBase

        #region ZOROSControllerInterface

        public string ControllerName {
            get { return "arm_controller"; }
        }
        public string ControllerType {
            get { return "position_controllers/JointTrajectoryController"; }
        }

        public string HardwareInterface {
            get { return "hardware_interface::PositionJointInterface"; }
        }

        ControllerStateEnum _state;
        public ControllerStateEnum ControllerState {
            get => _state;
            private set => _state = value;
        }

        public ControllerStateMessage ControllerStateMessage {
            get {

                ControllerStateMessage controllerStateMessage = new ControllerStateMessage {
                    name = ControllerName,
                    state = ControllerState.ToString().ToLower(),
                    type = ControllerType,
                    claimed_resources = new HardwareInterfaceResourcesMessage[1] {
                            new HardwareInterfaceResourcesMessage {
                                hardware_interface = HardwareInterface,
                                resources = new string[6] {
                                        "elbow_joint",
                                        "shoulder_lift_joint",
                                        "shoulder_pan_joint",
                                        "wrist_1_joint",
                                        "wrist_2_joint",
                                        "wrist_3_joint"
                                }
                            }
                        }
                };

                return controllerStateMessage;
            }
        }

        public void Load() {
            Debug.Log("INFO: ZOArmController::Load");
            Initialize();
        }

        public void Unload() {
            Debug.Log("INFO: ZOArmController::Unload");
            Terminate();
        }

        #endregion // ZOROSControllerInterface



        private void Initialize() {
            Debug.Log("INFO: ZOArmController::Initialize");

            // start up the follow joint trajectory action server
            _actionServer.ROSTopic = "/arm_controller/follow_joint_trajectory";
            _actionServer.Name = "arm_controller";
            _actionServer.Initialize();



            // advertise
            // ROSBridgeConnection.Advertise(ROSTopic, _jointStatesMessage.MessageType);

            // subscribe to the /arm_controller/command
            ROSBridgeConnection.Subscribe<JointTrajectoryMessage>("arm_controller", "/arm_controller/command", JointTrajectoryMessage.Type, OnControlMessageReceived);

            ControllerState = ControllerStateEnum.Running;


        }

        private void Terminate() {
            _actionServer.Terminate();
            ROSBridgeConnection?.Unsubscribe("arm_controller", "/arm_controller/command");

            ControllerState = ControllerStateEnum.Initialized;

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
            // Initialize();

        }

        public override void OnROSBridgeDisconnected(object rosUnityManager) {
            Debug.Log("INFO: ZOArmController::OnROSBridgeDisconnected");
            Terminate();
        }

        #endregion // ZOROSUnityInterface

    }
}