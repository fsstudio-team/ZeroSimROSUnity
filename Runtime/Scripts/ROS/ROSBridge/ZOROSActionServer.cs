using System.Linq;
using UnityEngine;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using ZO.Util;
using ZO.ROS.Unity;
using ZO.ROS.MessageTypes;
using ZO.ROS.MessageTypes.ActionLib;
using ZO.ROS.MessageTypes.ROSAPI;
using ZO.ROS.MessageTypes.Std;
using Newtonsoft.Json;

namespace ZO.ROS {

    /// <summary>
    /// The actionlib stack provides a standardized interface for interfacing with preemptable tasks. 
    /// Examples of this include moving the base to a target location, performing a laser scan and 
    /// returning the resulting point cloud, detecting the handle of a door, etc.
    /// <see>https://github.com/ros/actionlib/blob/noetic-devel/actionlib/include/actionlib/server/simple_action_server.h</see>
    /// </summary>
    /// <typeparam name="TActionGoal"></typeparam>
    /// <typeparam name="TActionResult"></typeparam>
    /// <typeparam name="TActionFeedback"></typeparam>
    public class ZOROSActionServer<TActionMessage, TGoalMessage> : ZOROSUnityInterface
                            where TActionMessage : ZOROSActionMessageInterface, new()
                            where TGoalMessage : ZOROSActionGoalMessageInterface, new() {

        public string _ROSTopic = "";

        /// <summary>
        /// The ROS Topic.  For example "/zerosim/joint_states"
        /// </summary>
        /// <value></value>
        public string ROSTopic {
            get => _ROSTopic;
            set => _ROSTopic = value;
        }

        public string _name;

        /// <summary>
        /// Unique name of the object.  For example: "joint.hinge_from_left_wheel"
        /// </summary>
        /// <value></value>
        public string Name {
            get {
                return _name;
            }

            set => _name = value;
        }



        /// <summary>
        /// The ROS Bridge singleton shortcut access.
        /// </summary>
        /// <value></value>
        protected ZOROSBridgeConnection ROSBridgeConnection {
            get { return ZOROSBridgeConnection.Instance; }
        }

        /// <summary>
        /// The ROS Unity Manger singleton shortcut access.
        /// </summary>
        /// <value></value>
        protected ZOROSUnityManager ROSUnityManager {
            get { return ZOROSUnityManager.Instance; }
        }


        /// <summary>
        /// On goal action received delegate.
        /// </summary>
        /// <value></value>
        public Func<ZOROSActionServer<TActionMessage, TGoalMessage>, TGoalMessage, Task> OnGoalReceived {
            get; set;
        }

        public Func<ZOROSActionServer<TActionMessage, TGoalMessage>, GoalIDMessage, Task> OnCancelReceived {
            get; set;
        }



        public TGoalMessage CurrentGoal {
            get;
            protected set;
        }

        public TGoalMessage NextGoal {
            get;
            protected set;
        }


        // This is defined according to actionlib_msgs/GoalStatus
        private ActionStatusEnum _currentGoalActionStatus = ActionStatusEnum.NO_GOAL;
        private ActionStatusEnum _nextGoalActionStatus = ActionStatusEnum.NO_GOAL;
        private GoalStatusArrayMessage _statusArrayMessage = new GoalStatusArrayMessage();

        /// <summary>
        /// The current goal action status.  Setting this will set and publish the status of the current goal.
        /// </summary>
        /// <value></value>
        public ActionStatusEnum CurrentGoalActionStatus {
            get => _currentGoalActionStatus;


            /// when the action state is set the status is published on the status topic
            protected set {
                _currentGoalActionStatus = value;
                List<GoalStatusMessage> goalStatuses = new List<GoalStatusMessage>();
                if (_currentGoalActionStatus != ActionStatusEnum.NO_GOAL) {
                    goalStatuses.Append(new GoalStatusMessage {
                        goal_id = CurrentGoal.goal_id,
                        status = (byte)_currentGoalActionStatus,
                        text = _actionStatusText
                    });
                }
                if (_nextGoalActionStatus != ActionStatusEnum.NO_GOAL) {
                    goalStatuses.Append(new GoalStatusMessage {
                        goal_id = NextGoal.goal_id,
                        status = (byte)_nextGoalActionStatus,
                        text = _actionStatusText
                    });

                }
                _statusArrayMessage.status_list = goalStatuses.ToArray();
                _statusArrayMessage.Update();
                ROSBridgeConnection.Publish<GoalStatusArrayMessage>(
                   _statusArrayMessage,
                    ROSTopic + "/status"
                );

            }
        }

        /// <summary>
        /// The current goal action status.  Setting this will set and publish the status of the current goal.
        /// </summary>
        /// <value></value>
        public ActionStatusEnum NextGoalActionStatus {
            get => _nextGoalActionStatus;


            /// when the action state is set the status is published on the status topic
            protected set {
                _nextGoalActionStatus = value;
                List<GoalStatusMessage> goalStatuses = new List<GoalStatusMessage>();
                if (_currentGoalActionStatus != ActionStatusEnum.NO_GOAL) {
                    goalStatuses.Append(new GoalStatusMessage {
                        goal_id = CurrentGoal.goal_id,
                        status = (byte)_currentGoalActionStatus,
                        text = _actionStatusText
                    });
                }
                if (_nextGoalActionStatus != ActionStatusEnum.NO_GOAL) {
                    goalStatuses.Append(new GoalStatusMessage {
                        goal_id = NextGoal.goal_id,
                        status = (byte)_nextGoalActionStatus,
                        text = _actionStatusText
                    });

                }

                _statusArrayMessage.status_list = goalStatuses.ToArray();

                _statusArrayMessage.Update();
                ROSBridgeConnection.Publish<GoalStatusArrayMessage>(
                   _statusArrayMessage,
                    ROSTopic + "/status"
                );
            }
        }

        private string _actionStatusText = "";
        public string ActionStatusText {
            get => _actionStatusText;
            protected set => _actionStatusText = value;
        }

        /// <summary>
        /// Will connect to ROS bridge connect and disconnect events.
        /// </summary>
        public void Initialize() {
            Debug.Log("INFO: ZOROSActionServer::Initialize");
            TActionMessage tmpActionMessage = new TActionMessage(); // used just to get message types
            // Advertise Action feedback
            ROSBridgeConnection.Advertise(ROSTopic + "/status", GoalStatusArrayMessage.Type);
            ROSBridgeConnection.Advertise(ROSTopic + "/feedback", tmpActionMessage.FeedbackMessageType);
            ROSBridgeConnection.Advertise(ROSTopic + "/result", tmpActionMessage.ResultMessageType);

            // Subscribe to Action cancel and goal topices
            ROSBridgeConnection.Subscribe<GoalIDMessage>(Name, ROSTopic + "/cancel", GoalIDMessage.Type, _OnCancelReceived);
            ROSBridgeConnection.Subscribe<TGoalMessage>(Name, ROSTopic + "/goal", tmpActionMessage.GoalMessageType, _OnGoalReceived);



            CurrentGoalActionStatus = ActionStatusEnum.NO_GOAL;
            NextGoalActionStatus = ActionStatusEnum.NO_GOAL;

        }

        /// <summary>
        /// Will disconnect to ROS bridge connect and disconnect events.
        /// </summary>
        public void Terminate() {
            Debug.Log("INFO: ZOROSActionServer::Terminate");
            ROSBridgeConnection.UnAdvertise(ROSTopic + "/status");
            ROSBridgeConnection.UnAdvertise(ROSTopic + "/feedback");
            ROSBridgeConnection.UnAdvertise(ROSTopic + "/result");

            ROSBridgeConnection.Unsubscribe(Name, ROSTopic + "/cancel");
            ROSBridgeConnection.Unsubscribe(Name, ROSTopic + "/goal");

            CurrentGoal = default(TGoalMessage);
            NextGoal = default(TGoalMessage);
            _nextGoalActionStatus = ActionStatusEnum.NO_GOAL;
            _currentGoalActionStatus = ActionStatusEnum.NO_GOAL;

        }



        #region ZOROSUnityInterface
        public void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOROSActionServer::OnROSBridgeConnected");
        }

        public void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOROSActionServer::OnROSBridgeDisconnected");
        }
        #endregion // ZOROSUnityInterface




        private Task _OnGoalReceived(ZOROSBridgeConnection rosBridgeConnection, ZOROSMessageInterface msg) {

            Debug.Log("INFO: ZOROSActionServer::OnGoalReceived");

            // tee up the next goal and set it to pending
            NextGoal = (TGoalMessage)msg;
            NextGoalActionStatus = ActionStatusEnum.PENDING;

            // notify delegates that we have a new goal
            OnGoalReceived?.Invoke(this, NextGoal);

            return Task.CompletedTask;
        }

        private Task _OnCancelReceived(ZOROSBridgeConnection rosBridgeConnection, ZOROSMessageInterface msg) {
            
            GoalIDMessage goalID = msg as GoalIDMessage;
            OnCancelReceived?.Invoke(this, goalID);

            Debug.Log("INFO: ZOROSActionServer::OnCancelReceived for goal id: " + goalID.id);
            // switch (ActionStatus) {
            //     case ActionStatusEnum.PENDING:
            //         ActionStatus = ActionStatusEnum.RECALLING;
            //         OnCancelReceived?.Invoke(this, goalID);
            //         break;
            //     case ActionStatusEnum.ACTIVE:
            //         ActionStatus = ActionStatusEnum.PREEMPTING;
            //         OnCancelReceived?.Invoke(this, goalID);
            //         break;
            //     default:
            //         Debug.LogWarning("WARNING: Goal cannot be canceled in current state: " + _actionStatus.ToString());
            //         break;
            // }

            return Task.CompletedTask;
        }

        // Server Triggered Actions
        // protected abstract void OnGoalActive();

        /// <summary>
        /// Accepts a new goal when one is available. The status of this goal is set to active upon 
        /// acceptance, and the status of any previously active goal is set to preempted. Preempts 
        /// received for the new goal between checking if isNewGoalAvailable or invocation of a goal 
        /// callback and the acceptNewGoal call will not trigger a preempt callback. This means, 
        /// isPreemptRequested should be called after accepting the goal even for callback-based 
        /// implementations to make sure the new goal does not have a pending preempt request.
        /// <see>http://wiki.ros.org/actionlib#SimpleActionServer_Goal_Policies</see>
        /// </summary>
        public void AcceptNewGoal(TGoalMessage newGoal) {

            // if we are still active start the preempt process
            if (CurrentGoalActionStatus == ActionStatusEnum.ACTIVE) {
                Debug.Log("INFO: AcceptNewGoal preempting goal: " + CurrentGoal.goal_id.id);

                // first notify client about preemption
                CurrentGoalActionStatus = ActionStatusEnum.PREEMPTING;

                // notify delegates about cancelattion
                OnCancelReceived?.Invoke(this, CurrentGoal.goal_id);

                // set next goal as the new goal
                NextGoal = newGoal;
                NextGoalActionStatus = ActionStatusEnum.PENDING;
            }


            // flip over next goal to current goal and notify client that we are active
            if (NextGoalActionStatus == ActionStatusEnum.PENDING) {
                CurrentGoal = NextGoal;
                NextGoal = default(TGoalMessage);
                _nextGoalActionStatus = ActionStatusEnum.NO_GOAL;
                CurrentGoalActionStatus = ActionStatusEnum.ACTIVE;

                Debug.Log("INFO: AcceptNewGoal accepting goal: " + CurrentGoal.goal_id.id);
            }

        }

        /// <summary>
        /// Set the status of the NextGoal to rejected.
        /// depending on what the current status of the goal is
        /// </summary>
        /// <param name="reason"></param>
        public void SetRejected(string reason = "") {
            ActionStatusText = reason;
            switch (NextGoalActionStatus) {
                case ActionStatusEnum.PENDING:
                    NextGoalActionStatus = ActionStatusEnum.REJECTED;
                    break;
                case ActionStatusEnum.RECALLING:
                    NextGoalActionStatus = ActionStatusEnum.REJECTED;
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot be rejected in current state: " + CurrentGoalActionStatus.ToString());
                    break;
            }

            NextGoal = default(TGoalMessage);
            _nextGoalActionStatus = ActionStatusEnum.NO_GOAL;

        }


        /// <summary>
        /// Set the current goal to succeeded.
        /// </summary>
        /// <param name="result">Optional result message</param>
        /// <param name="info">Optional additional info</param>
        public void SetSucceeded(ZOROSMessageInterface result = null, string info = "") {
            Debug.Log("INFO: Action goal succeeded.");
            ActionStatusText = info;
            switch (CurrentGoalActionStatus) {
                case ActionStatusEnum.ACTIVE:
                    CurrentGoalActionStatus = ActionStatusEnum.SUCCEEDED;
                    if (result != null) {
                        PublishResult(result);
                    }

                    break;
                case ActionStatusEnum.PREEMPTING:
                    CurrentGoalActionStatus = ActionStatusEnum.SUCCEEDED;
                    if (result != null) {
                        PublishResult(result);
                    }
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot succeed in current state: " + CurrentGoalActionStatus.ToString());
                    break;
            }

            CurrentGoalActionStatus = ActionStatusEnum.NO_GOAL;
        }


        /// <summary>
        /// Set status of the current goal to aborted.
        /// </summary>
        /// <param name="reason">Optional string explaining the reason for the abort.</param>
        public void SetAborted(string reason = "") {
            ActionStatusText = reason;
            switch (CurrentGoalActionStatus) {
                case ActionStatusEnum.ACTIVE:
                    CurrentGoalActionStatus = ActionStatusEnum.ABORTED;
                    break;
                case ActionStatusEnum.PREEMPTING:
                    CurrentGoalActionStatus = ActionStatusEnum.ABORTED;
                    break;
                default:
                    Debug.LogWarning("WARNING Goal cannot be aborted in current state: " + CurrentGoalActionStatus.ToString());
                    break;
            }

            CurrentGoal = default(TGoalMessage);
            _currentGoalActionStatus = ActionStatusEnum.NO_GOAL;

        }



        /// <summary>
        /// Set current goal to recalled or preempted 
        /// </summary>
        /// <param name="result"></param>
        public void SetCanceled(ZOROSMessageInterface result = null) {
            switch (CurrentGoalActionStatus) {
                case ActionStatusEnum.RECALLING:
                    CurrentGoalActionStatus = ActionStatusEnum.RECALLED;
                    break;
                case ActionStatusEnum.PREEMPTING:
                    CurrentGoalActionStatus = ActionStatusEnum.PREEMPTED;
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot be be canceled in current status state: " + CurrentGoalActionStatus.ToString());
                    return;
            }
        }



        /// <summary>
        /// Send feedback through the '/feedback' topic.
        /// </summary>
        /// <param name="feedback"></param>
        /// <typeparam name="T"></typeparam>
        public void PublishFeedback<T>(T feedback) where T : ZOROSMessageInterface {
            ROSBridgeConnection.Publish<T>(feedback, ROSTopic + "/feedback");
        }


        /// <summary>
        /// Publish the final result of the action.
        /// </summary>
        /// <param name="result"></param>
        /// <typeparam name="T"></typeparam>
        protected void PublishResult<T>(T result) where T : ZOROSMessageInterface {
            ROSBridgeConnection.Publish<T>(result, ROSTopic + "/result");
        }
    }

}