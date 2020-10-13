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
using ZO.ROS.MessageTypes.ActionLib;
using ZO.Physics;
using ZO.ROS.Unity;
using ZO.ROS.Unity.Service;
using ZO.ROS.Publisher;
using ZO.Document;


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
        /// The ROS Action server that handles arm movement goal messaging. (readonly)
        /// </summary>
        /// <value></value>
        public ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal> ActionServer {
            get => _actionServer;
        }

        private FollowJointTrajectoryActionMessage _followJointTrajectorActionMessage = new FollowJointTrajectoryActionMessage();

        /// <summary>
        /// The FollowJointTrajectoryActionMessage which contains the goal, result & feedback messages.
        /// </summary>
        /// <value></value>
        public FollowJointTrajectoryActionMessage ActionMessage {
            get => _followJointTrajectorActionMessage;
        }


        /// <summary>
        /// Simple single joint control message as used by RQT joint controller.  NOT by MoveIt.
        /// </summary>
        /// <returns></returns>
        private JointTrajectoryMessage _commandMessage = new JointTrajectoryMessage();


        private bool _needToProcessCommandMessage = false;



        /// <summary>
        /// Joint state publisher message published on `/arm_controller/state` topic.
        /// </summary>
        /// <returns></returns>
        private JointTrajectoryControllerStateMessage _trajectoryControllerStateMessage = new JointTrajectoryControllerStateMessage();



        /// <summary>
        /// Current joint trajectory move goal. (readonly)
        /// </summary>
        /// <value>FollowJointTrajectoryActionGoal (readonly)</value>
        private FollowJointTrajectoryActionGoal Goal {
            get => ActionServer.CurrentGoal;
        }


        /// <summary>
        /// The current goal status. 
        /// </summary>
        /// <value></value>
        private ActionStatusEnum GoalStatus {
            get => ActionServer.CurrentGoalActionStatus;
        }



        private float _currentGoalTime = 0;
        private JointTrajectoryPointMessage _currentPoint = new JointTrajectoryPointMessage();


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

        protected override void ZOReset() {
            base.ZOReset();
            UpdateRateHz = 25.0f;
            ROSTopic = "/arm_controller";
        }


        protected override void ZOAwake() {
            Name = gameObject.name + "_arm_controller";
        }

        protected override void ZOStart() {
            Debug.Log("INFO: ZOArmController::ZOStart");
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

            _trajectoryControllerStateMessage.joint_names = jointNames;
            ActionMessage.action_feedback.feedback.desired.positions = new double[joints.Length];
            ActionMessage.action_feedback.feedback.desired.velocities = new double[joints.Length];
            ActionMessage.action_feedback.feedback.desired.accelerations = new double[joints.Length];
            ActionMessage.action_feedback.feedback.actual.positions = new double[joints.Length];
            ActionMessage.action_feedback.feedback.actual.velocities = new double[joints.Length];
            ActionMessage.action_feedback.feedback.error.positions = new double[joints.Length];
            ActionMessage.action_feedback.feedback.error.velocities = new double[joints.Length];


            // register with controller manager
            ControllerManager.RegisterController(this);

            if (ZOROSBridgeConnection.Instance.IsConnected) {
                Initialize();
            }
        }


        protected override void ZOFixedUpdateHzSynchronized() {

            // if not connected the don't run
            if (ZOROSBridgeConnection.Instance.IsConnected == false) {
                return;
            }

            if (this.ControllerState == ControllerStateEnum.Running) {

                // see if we have any goals and update the controller state message
                if (GoalStatus == ActionStatusEnum.ACTIVE) {

                    // find the closest target position given the current time
                    bool foundTargetPoint = false;
                    foreach (JointTrajectoryPointMessage point in Goal.goal.trajectory.points) {
                        // Debug.Log("INFO: point time: " + point.time_from_start.Seconds.ToString("R3"));
                        if (_currentGoalTime <= point.time_from_start.Seconds) {
                            _currentPoint = point;
                            foundTargetPoint = true;
                            break;
                        }
                    }


                    if (foundTargetPoint == true) {

                        for (int i = 0; i < Goal.goal.trajectory.joint_names.Length; i++) {
                            _trajectoryControllerStateMessage.joint_names[i] = Goal.goal.trajectory.joint_names[i];
                            ZOJointInterface joint = GetJointByName(Goal.goal.trajectory.joint_names[i]);

                            _trajectoryControllerStateMessage.desired.positions[i] = _currentPoint.positions[i];
                            ActionMessage.action_feedback.feedback.desired.positions[i] = _currentPoint.positions[i];

                            _trajectoryControllerStateMessage.actual.positions[i] = joint.Position;
                            ActionMessage.action_feedback.feedback.actual.positions[i] = joint.Position;
                            _trajectoryControllerStateMessage.actual.velocities[i] = joint.Velocity;
                            ActionMessage.action_feedback.feedback.actual.velocities[i] = joint.Velocity;

                            _trajectoryControllerStateMessage.error.positions[i] = joint.Position - _trajectoryControllerStateMessage.desired.positions[i];
                            ActionMessage.action_feedback.feedback.error.positions[i] = joint.Position - _trajectoryControllerStateMessage.desired.positions[i];
                            // _trajectoryControllerStateMessage.error.velocities[i] = joint.Velocity - _trajectoryControllerStateMessage.desired.velocities[i];

                            joint.Position = (float)_trajectoryControllerStateMessage.desired.positions[i];

                        }

                    } else {
                        // at the end of the points for this goal so finish it
                        ActionMessage.action_result.status = new GoalStatusMessage(Goal.goal_id, (byte)ActionStatusEnum.SUCCEEDED, "finished arm control");
                        ActionMessage.action_result.result = new FollowJointTrajectoryResult(FollowJointTrajectoryResult.SUCCESSFUL);
                        ActionMessage.Update();
                        ActionServer.SetSucceeded(ActionMessage.action_result, "Finished arm control movement");

                    }

                    // update time
                    _currentGoalTime += UpdateTimeSeconds;

                }

                // process the "simple" joint command message (as used by RQT but NOT MoveIt)
                if (_needToProcessCommandMessage == true) {
                    int i = 0;
                    foreach (ZOJointInterface joint in Joints) {
                        _trajectoryControllerStateMessage.actual.positions[i] = joint.Position;
                        ActionMessage.action_feedback.feedback.actual.positions[i] = joint.Position;
                        _trajectoryControllerStateMessage.actual.velocities[i] = joint.Velocity;
                        ActionMessage.action_feedback.feedback.actual.velocities[i] = joint.Velocity;

                        _trajectoryControllerStateMessage.error.positions[i] = joint.Position - _trajectoryControllerStateMessage.desired.positions[i];
                        ActionMessage.action_feedback.feedback.error.positions[i] = joint.Position - _trajectoryControllerStateMessage.desired.positions[i];
                        // _trajectoryControllerStateMessage.error.velocities[i] = joint.Velocity - _trajectoryControllerStateMessage.desired.velocities[i];

                        joint.Position = (float)_trajectoryControllerStateMessage.desired.positions[i];
                        i++;

                        // BUGBUG: we should be waiting until we get within some error of the desired position or we timed out
                        _needToProcessCommandMessage = false;
                    }

                }

                // update the joint states
                _trajectoryControllerStateMessage.Update();
                ActionMessage.action_feedback.Update();


                //TODO:  this is for the simple rqt control

                // publish feed back on ROS topic
                ROSBridgeConnection.Publish<JointTrajectoryControllerStateMessage>(_trajectoryControllerStateMessage, ControllerManager.Name + "/arm_controller/state");

                // publish feedback on ROS action
                _actionServer.PublishFeedback<FollowJointTrajectoryActionFeedback>(ActionMessage.action_feedback);

            }

        }

        protected override void ZOOnDestroy() {
            base.ZOOnDestroy();
            Terminate();
        }

        protected override void ZOOnGUI() {
            base.ZOOnGUI();
            int y = 10;
            GUI.TextField(new Rect(10, y, 200, 20), "Goal Status: " + GoalStatus.ToString());
            y += 20;

            for (int i = 0; i < _trajectoryControllerStateMessage.joint_names.Length; i++, y += 25) {
                ZOJointInterface joint = GetJointByName(_trajectoryControllerStateMessage.joint_names[i]);

                GUI.TextField(new Rect(10, y, 500, 22), _trajectoryControllerStateMessage.joint_names[i]
                //+ " actual: " + (_trajectoryControllerStateMessage.actual.positions[i] * Mathf.Rad2Deg).ToString("N2")
                + " actual: " + (joint.Position * Mathf.Rad2Deg).ToString("N2")
                + " desired: " + (_trajectoryControllerStateMessage.desired.positions[i] * Mathf.Rad2Deg).ToString("N2")
                + " error: " + (_trajectoryControllerStateMessage.error.positions[i] * Mathf.Rad2Deg).ToString("N2"));
            }
            if (GoalStatus == ActionStatusEnum.ACTIVE) {
                foreach (JointTrajectoryPointMessage point in Goal.goal.trajectory.points) {
                    y += 25;
                    GUI.TextField(new Rect(10, y, 200, 22), "Point time: " + point.time_from_start.Seconds.ToString("R2"));
                }

            }

            y += 25;
            GUI.TextField(new Rect(10, y, 200, 22), "Current time: " + _currentGoalTime.ToString("R2"));

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
        /// Joints must implement the ZOJointInterface.  Fixed joints are ignored.
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

        public ZOJointInterface GetJointByName(string name) {
            foreach (ZOJointInterface joint in Joints) {
                if (joint.Name == name) {
                    return joint;
                }
            }
            return null;
        }



        /// <summary>
        /// Build a ControllerStateMessage for the controller manager.
        /// Basically just lists all the joints for this arm controller.
        /// </summary>
        /// <value></value>
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
        public void LoadController() {
            Debug.Log("INFO: ZOArmController::Load");
            Initialize();   // BUGBUG: we should separate initialized into load -> start calls 
            ControllerState = ControllerStateEnum.Initialized;
        }

        /// <summary>
        /// Implements ROS Controller Unload
        /// </summary>
        public void UnloadController() {
            Debug.Log("INFO: ZOArmController::Unload");
            Terminate();
            ControllerState = ControllerStateEnum.Terminated;
        }

        public void StartController() {
            Debug.Log("INFO: ZOArmController::Start");
            ControllerState = ControllerStateEnum.Running;
            _actionServer.OnGoalReceived += OnActionGoalReceived;
            _actionServer.OnCancelReceived += OnActionCancelReceived;

        }

        public void StopController() {
            Debug.Log("INFO: ZOArmController::Stop");
            ControllerState = ControllerStateEnum.Stopped;
            _actionServer.OnGoalReceived -= OnActionGoalReceived;
            _actionServer.OnCancelReceived -= OnActionCancelReceived;
        }

        #endregion // ZOROSControllerInterface


        // BUGBUG: we should separate initialized into load -> start calls 
        private void Initialize() {
            Debug.Log("INFO: ZOArmController::Initialize");

            // start up the follow joint trajectory action server
            // _actionServer.ROSTopic = "/arm_controller/follow_joint_trajectory"; //BUGBUG: HARDWIRED
            _actionServer.ROSTopic = ROSTopic;
            _actionServer.Name = "arm_controller";
            // _actionServer.OnGoalReceived += OnActionGoalReceived;
            // _actionServer.OnCancelReceived += OnActionCancelReceived;
            _actionServer.Initialize();




            // subscribe to the /arm_controller/command
            // This is a "simple" interface NOT used by MoveIt but something like a simple joint controller you would find using RQT
            ROSBridgeConnection.Subscribe<JointTrajectoryMessage>("arm_controller", ControllerManager.Name + "/arm_controller/command", JointTrajectoryMessage.Type, OnControlMessageReceived);

            // advertise joint state
            ROSBridgeConnection.Advertise(ControllerManager.Name + "/arm_controller/state", JointTrajectoryControllerStateMessage.Type);

            

        }

        private void Terminate() {
            Debug.Log("INFO: ZOArmController::Terminate");

            ROSBridgeConnection?.Unsubscribe("arm_controller", ControllerManager?.Name + "/arm_controller/command");
            ROSBridgeConnection?.UnAdvertise(ControllerManager?.Name + "/arm_controller/state");

            ControllerState = ControllerStateEnum.Stopped;

            _actionServer.OnGoalReceived -= OnActionGoalReceived;
            _actionServer.OnCancelReceived -= OnActionCancelReceived;

            _actionServer.Terminate();
        }

        #region Control Message/Action Handlers


        /// <summary>
        /// This responds to a "simple" JointTrajectoryMessage. This is usually seen in the simple RQT
        /// joint controller but NOT used by MoveIt.
        /// </summary>
        /// <param name="rosBridgeConnection"></param>
        /// <param name="msg"></param>
        /// <returns></returns>
        public Task OnControlMessageReceived(ZOROSBridgeConnection rosBridgeConnection, ZOROSMessageInterface msg) {
            _commandMessage = (JointTrajectoryMessage)msg;
            // BUGBUG: assuming that we only get a single points message when we can get a whole array making
            // up a points path.
            _trajectoryControllerStateMessage.desired = _commandMessage.points[0];
            // Debug.Log("INFO: command message: " + JsonConvert.SerializeObject(_commandMessage));

            _needToProcessCommandMessage = true;

            return Task.CompletedTask;
        }


        /// <summary>
        /// This is a FollowJointTrajectoryActionMessage action responder.  Usually used by MoveIt.
        /// </summary>
        /// <param name="actionServer"></param>
        /// <param name="goalMessage"></param>
        /// <returns></returns>
        Task OnActionGoalReceived(ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal> actionServer, FollowJointTrajectoryActionGoal goalMessage) {

            Debug.Log("INFO: ZOArmController::OnGoalReceived");

            // automatically acccept new goal
            // TODO: perhaps more complex logic?  are there conditions where we will not accept a new goal?
            actionServer.AcceptNewGoal(goalMessage);

            _currentGoalTime = 0;

            return Task.CompletedTask;
        }

        Task OnActionCancelReceived(ZOROSActionServer<FollowJointTrajectoryActionMessage, FollowJointTrajectoryActionGoal> actionServer, GoalIDMessage goalID) {

            Debug.Log("INFO: ZOArmController::OnCancelReceived");
            // confirm cancellation to the action server
            actionServer.SetCanceled();

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