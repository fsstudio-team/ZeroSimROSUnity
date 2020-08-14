using System.Linq;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes;
using ZO.ROS.MessageTypes.Control;
using ZO.ROS.MessageTypes.Trajectory;
using ZO.ROS.MessageTypes.ControllerManager;
using ZO.Physics;
using ZO.ROS.Unity;
using ZO.ROS.Unity.Service;


namespace ZO.ROS.Controllers {
    public class ZOArmController : ZOROSUnityGameObjectBase, ZOROSControllerInterface {

        /// <summary>
        /// Arm controller action server.
        /// </summary>
        /// <typeparam name="FollowJointTrajectoryActionMessage"></typeparam>
        /// <typeparam name="FollowJointTrajectoryActionGoal"></typeparam>
        /// <returns></returns>
        private ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal> _actionServer = new ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal>();
        private JointTrajectoryMessage _commandMessage = new JointTrajectoryMessage();
        private JointTrajectoryControllerStateMessage _trajectoryControllerStateMessage = new JointTrajectoryControllerStateMessage();


        private ZOControllerManagerService _controllerManager = null;
        private ZOControllerManagerService ControllerManager {
            get {
                if (_controllerManager == null) {
                    ZOControllerManagerService[] controllerManagers = this.GetComponents<ZOControllerManagerService>();
                    if (controllerManagers.Length != 1) {  // there can only be one PER robot
                        Debug.LogError("ERROR: can only have one ZOControllerManagerService per ZODocumentRoot: " + controllerManagers.Length.ToString());
                        return null;
                    }
                    _controllerManager = controllerManagers[0];
                }
                return _controllerManager;
            }
        }

        #region ZOGameObjectBase

        protected override void ZOAwake() {
            Name = "arm_controller";
        }
        protected override void ZOStart() {
            base.ZOStart();

            // preload joints because they cannot be updated outside the main unity thread
            ZOJointInterface[] joints = Joints;
            string[] jointNames = JointNames;

            // initialize the joint state message
            _trajectoryControllerStateMessage.joint_names = jointNames;
            _trajectoryControllerStateMessage.desired.positions = new double[joints.Length];
            _trajectoryControllerStateMessage.desired.velocities = new double[joints.Length];
            _trajectoryControllerStateMessage.desired.accelerations = new double[joints.Length];
            _trajectoryControllerStateMessage.actual.positions = new double[joints.Length];
            _trajectoryControllerStateMessage.actual.velocities = new double[joints.Length];
            _trajectoryControllerStateMessage.error.positions = new double[joints.Length];
            _trajectoryControllerStateMessage.error.velocities = new double[joints.Length];

            // register with controller manager
            ControllerManager.RegisterController(this);

            if (ZOROSBridgeConnection.Instance.IsConnected) {
                Initialize();
            }
        }

        protected override void ZOFixedUpdateHzSynchronized() {
            if (this.ControllerState == ControllerStateEnum.Running) {
                // update the joint states
                _trajectoryControllerStateMessage.Update();
                int i = 0;
                foreach (ZOJointInterface joint in Joints) {
                    _trajectoryControllerStateMessage.actual.positions[i] = joint.Position;
                    _trajectoryControllerStateMessage.actual.velocities[i] = joint.Velocity;

                    _trajectoryControllerStateMessage.error.positions[i] = joint.Position - _trajectoryControllerStateMessage.desired.positions[i];
                    // _trajectoryControllerStateMessage.error.velocities[i] = joint.Velocity - _trajectoryControllerStateMessage.desired.velocities[i];

                    joint.Position = (float)_trajectoryControllerStateMessage.desired.positions[i];
                    i++;
                }

                ROSBridgeConnection.Publish<JointTrajectoryControllerStateMessage>(_trajectoryControllerStateMessage, ControllerManager.Name + "/arm_controller/state");

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

        private ZOJointInterface[] _joints = null;

        /// <summary>
        /// Get all the joints that are children of this arm controller.  
        /// Joints must implement the ZOJointInterface
        /// </summary>
        /// <value></value>
        public ZOJointInterface[] Joints {
            get {
                if (_joints == null) {     
                    // Get all the joints but filter out fixed joints               
                    _joints = Array.FindAll(this.transform.GetComponentsInChildren<ZOJointInterface>(), 
                                joint => joint.Type != "joint.articulated_body.fixedjoint");
                }
                return _joints;
            }
        }

        private string[] _jointNames;
        /// <summary>
        /// Get an array of Joint names.
        /// </summary>
        /// <value></value>
        public string[] JointNames {
            get {
                if (_jointNames == null) {
                    _jointNames = new string[Joints.Length];
                    for (int i = 0; i < Joints.Length; i++) {
                        _jointNames[i] = Joints[i].Name;
                    }
                }
                return _jointNames;

            }
        }



        public ControllerStateMessage ControllerStateMessage {
            get {

                HardwareInterfaceResourcesMessage[] claimed_resources = new HardwareInterfaceResourcesMessage[1];
                claimed_resources[0] = new HardwareInterfaceResourcesMessage(HardwareInterface, JointNames);
                ControllerStateMessage controllerStateMessage = new ControllerStateMessage(ControllerName, ControllerState.ToString().ToLower(), ControllerType, claimed_resources);

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
            ROSBridgeConnection.Subscribe<JointTrajectoryMessage>("arm_controller", ControllerManager.Name + "/arm_controller/command", JointTrajectoryMessage.Type, OnControlMessageReceived);

            // advertise joint state
            ROSBridgeConnection.Advertise(ControllerManager.Name + "/arm_controller/state", JointTrajectoryControllerStateMessage.Type);

            ControllerState = ControllerStateEnum.Running;


        }

        private void Terminate() {
            ROSBridgeConnection?.Unsubscribe("arm_controller", ControllerManager?.Name + "/arm_controller/command");
            ROSBridgeConnection?.UnAdvertise(ControllerManager?.Name + "/arm_controller/state");

            ControllerState = ControllerStateEnum.Stopped;

            _actionServer.Terminate();
        }

        public Task OnControlMessageReceived(ZOROSBridgeConnection rosBridgeConnection, ZOROSMessageInterface msg) {
            _commandMessage = (JointTrajectoryMessage)msg;
            _trajectoryControllerStateMessage.desired = _commandMessage.points[0];
            // Debug.Log("INFO: command message: " + JsonConvert.SerializeObject(_commandMessage));

            return Task.CompletedTask;
        }


        #region ZOSerializationInterface

        public string Type {
            get => "controller.arm_controller";
        }

        public override JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("update_rate_hz", UpdateRateHz)
            );
            JSON = json;
            return json;
        }

        public override void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            Name = json["name"].Value<string>();
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