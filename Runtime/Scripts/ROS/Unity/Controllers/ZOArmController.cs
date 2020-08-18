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
using ZO.ROS.Publisher;

namespace ZO.ROS.Controllers {

    /// <summary>
    /// Controller for executing joint-space trajectories on a group of joints. Trajectories are specified as
    /// a set of waypoints to be reached at specific time instants, which the controller attempts to execute
    /// as well as the mechanism allows. Waypoints consist of positions, and optionally velocities and 
    /// accelerations. 
    /// </summary>
    [RequireComponent(typeof(ZOROSJointStatesPublisher))]
    public class ZOArmController : ZOROSUnityGameObjectBase, ZOROSControllerInterface {

        /// <summary>
        /// Arm controller action server as used by MoveIt
        /// </summary>
        /// <typeparam name="FollowJointTrajectoryActionMessage"></typeparam>
        /// <typeparam name="FollowJointTrajectoryActionGoal"></typeparam>
        /// <returns></returns>
        private ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal> _actionServer = new ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal>();

        /// <summary>
        /// Simple single joint control message as used by RQT joint controller.
        /// </summary>
        /// <returns></returns>
        private JointTrajectoryMessage _commandMessage = new JointTrajectoryMessage();

        /// <summary>
        /// Joint state publisher message published on `/arm_controller/state` topic.
        /// </summary>
        /// <returns></returns>
        private JointTrajectoryControllerStateMessage _trajectoryControllerStateMessage = new JointTrajectoryControllerStateMessage();


        /// <summary>
        /// Joint state feeedback message for the ROS action server.
        /// </summary>
        /// <returns></returns>
        private FollowJointTrajectoryActionFeedback _followJointTrajectoryActionFeedback = new FollowJointTrajectoryActionFeedback();


        /// <summary>
        /// Action interface goal queue
        /// </summary>
        /// <value></value>
        private Queue<FollowJointTrajectoryActionGoal> Goals {
            get; set;
        }


        /// <summary>
        /// Controller manager manages all ROS controllers on a single robot such as this arm controller.  
        /// Important: this is per robot and not global.
        /// </summary>
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
            Name = gameObject.name + "_arm_controller";
        }

        protected override void ZOStart() {
            base.ZOStart();

            // setup action goals queue
            Goals = new Queue<FollowJointTrajectoryActionGoal>();

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

            _trajectoryControllerStateMessage.joint_names = jointNames;
            _followJointTrajectoryActionFeedback.feedback.desired.positions = new double[joints.Length];
            _followJointTrajectoryActionFeedback.feedback.desired.velocities = new double[joints.Length];
            _followJointTrajectoryActionFeedback.feedback.desired.accelerations = new double[joints.Length];
            _followJointTrajectoryActionFeedback.feedback.actual.positions = new double[joints.Length];
            _followJointTrajectoryActionFeedback.feedback.actual.velocities = new double[joints.Length];
            _followJointTrajectoryActionFeedback.feedback.error.positions = new double[joints.Length];
            _followJointTrajectoryActionFeedback.feedback.error.velocities = new double[joints.Length];


            // register with controller manager
            ControllerManager.RegisterController(this);

            if (ZOROSBridgeConnection.Instance.IsConnected) {
                Initialize();
            }
        }

        protected override void ZOFixedUpdateHzSynchronized() {
            if (this.ControllerState == ControllerStateEnum.Running) {

                // see if we have any goals and update the controller state message
                if (Goals.Count > 0) {

                    FollowJointTrajectoryActionGoal currentGoal = Goals.Dequeue();
                    _commandMessage = currentGoal.goal.trajectory;
                    // update the desired position
                    _trajectoryControllerStateMessage.desired = _commandMessage.points[0];
                }

                // update the joint states
                _trajectoryControllerStateMessage.Update();
                _followJointTrajectoryActionFeedback.Update();


                int i = 0;
                foreach (ZOJointInterface joint in Joints) {
                    _trajectoryControllerStateMessage.actual.positions[i] = joint.Position;
                    _followJointTrajectoryActionFeedback.feedback.actual.positions[i] = joint.Position;
                    _trajectoryControllerStateMessage.actual.velocities[i] = joint.Velocity;
                    _followJointTrajectoryActionFeedback.feedback.actual.velocities[i] = joint.Velocity;

                    _trajectoryControllerStateMessage.error.positions[i] = joint.Position - _trajectoryControllerStateMessage.desired.positions[i];
                    _followJointTrajectoryActionFeedback.feedback.error.positions[i] = joint.Position - _trajectoryControllerStateMessage.desired.positions[i];
                    // _trajectoryControllerStateMessage.error.velocities[i] = joint.Velocity - _trajectoryControllerStateMessage.desired.velocities[i];

                    joint.Position = (float)_trajectoryControllerStateMessage.desired.positions[i];
                    i++;
                }

                // publish feed back on ROS topic
                ROSBridgeConnection.Publish<JointTrajectoryControllerStateMessage>(_trajectoryControllerStateMessage, ControllerManager.Name + "/arm_controller/state");

                // publish feedback on ROS action
                _actionServer.PublishFeedback<FollowJointTrajectoryActionFeedback>(_followJointTrajectoryActionFeedback);

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
        /// <summary>
        /// Returns controller state of Stopped, Initialize, or Running
        /// </summary>
        /// <value></value>
        public ControllerStateEnum ControllerState {
            get => _state;
            private set => _state = value;
        }


        // joints cache lazy loaded by Joints property
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
                                joint => joint.Type != "joint.articulated_body.fixedjoint"
                                        && joint.Type != "joint.fixed");
                }
                return _joints;
            }
        }

        private string[] _jointNames;
        /// <summary>
        /// Get an array of Joint names.
        /// </summary>
        /// /// <value></value>
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

        /// <summary>
        /// Implements ROS Controller Load
        /// </summary>
        public void Load() {
            Debug.Log("INFO: ZOArmController::Load");
            Initialize();
        }

        /// <summary>
        /// Implements ROS Controller Unload
        /// </summary>
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
            _actionServer.OnGoalReceived += OnGoalReceived;
            _actionServer.OnCancelReceived += OnCancelReceived;
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

        #region Control Message/Action Handlers


        /// <summary>
        /// This responds to a "simple" JointTrajectoryMessage. This is usually seen in the simple RViz
        /// joint controller but not used by MoveIt.
        /// </summary>
        /// <param name="rosBridgeConnection"></param>
        /// <param name="msg"></param>
        /// <returns></returns>
        public Task OnControlMessageReceived(ZOROSBridgeConnection rosBridgeConnection, ZOROSMessageInterface msg) {
            _commandMessage = (JointTrajectoryMessage)msg;
            _trajectoryControllerStateMessage.desired = _commandMessage.points[0];
            // Debug.Log("INFO: command message: " + JsonConvert.SerializeObject(_commandMessage));

            return Task.CompletedTask;
        }


        /// <summary>
        /// This is a FollowJointTrajectoryActionMessage action responder.  Usually used by MoveIt.
        /// </summary>
        /// <param name="actionServer"></param>
        /// <param name="goalMessage"></param>
        /// <returns></returns>
        Task OnGoalReceived(ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal> actionServer, FollowJointTrajectoryActionGoal goalMessage) {

            Debug.Log("INFO: ZOArmController::OnGoalReceived");

            // queue up the messages to be processed later.
            Goals.Enqueue(goalMessage);

            actionServer.AcceptNewGoal("hey ho lets go");

            return Task.CompletedTask;
        }

        Task OnCancelReceived(ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal> actionServer) {

            Debug.Log("INFO: ZOArmController::OnCancelReceived");

            return Task.CompletedTask;


        }

        #endregion


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
        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOArmController::OnROSBridgeConnected");
            // Initialize();

        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOArmController::OnROSBridgeDisconnected");
            Terminate();
        }

        #endregion // ZOROSUnityInterface

    }
}