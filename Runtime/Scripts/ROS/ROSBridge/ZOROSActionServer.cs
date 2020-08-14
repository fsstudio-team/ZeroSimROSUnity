using UnityEngine;
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
    /// </summary>
    /// <typeparam name="TActionGoal"></typeparam>
    /// <typeparam name="TActionResult"></typeparam>
    /// <typeparam name="TActionFeedback"></typeparam>
    public class ZOROSActionServer<TActionMessage, TGoalMessage> : ZOROSUnityInterface
                            where TActionMessage : ZOROSActionMessageInterface, new()
                            where TGoalMessage : ZOROSMessageInterface, new() {

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



        // This is defined according to actionlib_msgs/GoalStatus
        private ActionStatusEnum _actionStatus = ActionStatusEnum.NO_GOAL;
        public ActionStatusEnum ActionStatus {
            get => _actionStatus;


            /// when the action state is set the status is published on the status topic
            protected set {
                _actionStatus = value;
                // publish status
                if (_actionStatus == ActionStatusEnum.NO_GOAL) {
                    GoalStatusArrayMessage statusArrayMessage = new GoalStatusArrayMessage();
                    statusArrayMessage.Update();
                    ROSBridgeConnection.Publish<GoalStatusArrayMessage>(statusArrayMessage, ROSTopic + "/status");
                } else {
                    GoalStatusArrayMessage statusArrayMessage = new GoalStatusArrayMessage {
                        status_list = new GoalStatusMessage[]
                            {
                                new GoalStatusMessage {
                                    status = (byte)_actionStatus,
                                    text = _actionStatusText
                                }
                            }
                    };
                    statusArrayMessage.Update();
                    ROSBridgeConnection.Publish<GoalStatusArrayMessage>(
                       statusArrayMessage,
                        ROSTopic + "/status"
                    );
                }
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
            ROSBridgeConnection.Subscribe<GoalIDMessage>(Name, ROSTopic + "/cancel", GoalIDMessage.Type, OnCancelReceived);
            ROSBridgeConnection.Subscribe<TGoalMessage>(Name, ROSTopic + "/goal", tmpActionMessage.GoalMessageType, OnGoalReceived);


            ActionStatus = ActionStatusEnum.NO_GOAL;

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

        }



        #region ZOROSUnityInterface
        public void OnROSBridgeConnected(object rosUnityManager) {
            Debug.Log("INFO: ZOROSActionServer::OnROSBridgeConnected");
        }

        public void OnROSBridgeDisconnected(object rosUnityManager) {
            Debug.Log("INFO: ZOROSActionServer::OnROSBridgeDisconnected");
        }
        #endregion // ZOROSUnityInterface




        // Client Triggered Actions
        // When receive a new goal
        // protected abstract void OnGoalReceived(ZOROSMessageInterface actionGoal);
        private Task OnGoalReceived(ZOROSBridgeConnection oROSBridgeConnection, ZOROSMessageInterface msg) {

            Debug.Log("INFO: ZOROSActionServer::OnGoalReceived");

            ActionStatus = ActionStatusEnum.PENDING;
            // OnGoalReceived(msg);
            return Task.CompletedTask;
        }

        // When the goal is cancelled by the client
        // protected abstract void OnGoalRecalling(GoalIDMessage goalID);
        // protected abstract void OnGoalPreempting();
        private Task OnCancelReceived(ZOROSBridgeConnection oROSBridgeConnection, ZOROSMessageInterface msg) {
            Debug.Log("INFO: ZOROSActionServer::OnCancelReceived");
            GoalIDMessage goalID = msg as GoalIDMessage;
            switch (ActionStatus) {
                case ActionStatusEnum.PENDING:
                    ActionStatus = ActionStatusEnum.RECALLING;
                    // OnGoalRecalling(goalID);
                    break;
                case ActionStatusEnum.ACTIVE:
                    ActionStatus = ActionStatusEnum.PREEMPTING;
                    // OnGoalPreempting();
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot be canceled in current state: " + _actionStatus.ToString());
                    break;
            }

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
        /// </summary>
        /// <param name="info"></param>
        public void AcceptNewGoal(string info = "") {
            ActionStatusText = info;
            switch (ActionStatus) {
                case ActionStatusEnum.PENDING:
                    ActionStatus = ActionStatusEnum.ACTIVE;
                    // OnGoalActive();
                    break;
                case ActionStatusEnum.RECALLING:
                    ActionStatus = ActionStatusEnum.PREEMPTING;
                    // OnGoalPreempting();
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot be accepted in current state: " + ActionStatus.ToString());
                    break;
            }
        }

        // protected abstract void OnGoalRejected();
        public void SetRejected(string reason = "") {
            ActionStatusText = reason;
            switch (ActionStatus) {
                case ActionStatusEnum.PENDING:
                    ActionStatus = ActionStatusEnum.REJECTED;
                    // OnGoalRejected();
                    break;
                case ActionStatusEnum.RECALLING:
                    ActionStatus = ActionStatusEnum.REJECTED;
                    // OnGoalRejected();
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot be rejected in current state: " + ActionStatus.ToString());
                    break;
            }
        }

        // protected abstract void OnGoalSucceeded();
        public void SetSucceeded(ZOROSMessageInterface result = null, string info = "") {
            ActionStatusText = info;
            switch (ActionStatus) {
                case ActionStatusEnum.ACTIVE:
                    ActionStatus = ActionStatusEnum.SUCCEEDED;
                    PublishResult(result);
                    // OnGoalSucceeded();
                    break;
                case ActionStatusEnum.PREEMPTING:
                    ActionStatus = ActionStatusEnum.SUCCEEDED;
                    PublishResult(result);
                    // OnGoalSucceeded();
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot succeed in current state: " + ActionStatus.ToString());
                    break;
            }
        }

        // protected abstract void OnGoalAborted();
        public void SetAborted(string reason = "") {
            ActionStatusText = reason;
            switch (ActionStatus) {
                case ActionStatusEnum.ACTIVE:
                    ActionStatus = ActionStatusEnum.ABORTED;
                    // OnGoalAborted();
                    break;
                case ActionStatusEnum.PREEMPTING:
                    ActionStatus = ActionStatusEnum.ABORTED;
                    // OnGoalAborted();
                    break;
                default:
                    Debug.LogWarning("WARNING Goal cannot be aborted in current state: " + ActionStatus.ToString());
                    break;
            }
        }

        // protected abstract void OnGoalCanceled(ZOROSMessageInterface result);
        public void SetCanceled(ZOROSMessageInterface result = null) {
            switch (ActionStatus) {
                case ActionStatusEnum.RECALLING:
                    ActionStatus = ActionStatusEnum.RECALLED;
                    break;
                case ActionStatusEnum.PREEMPTING:
                    ActionStatus = ActionStatusEnum.PREEMPTED;
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot be be canceled in current status state: " + ActionStatus.ToString());
                    return;
            }
            // OnGoalCanceled(result);
        }

        // protected void UpdateAndPublishStatus(ActionStatus actionStatus, string text = "") {
        //     this._actionStatus = actionStatus;
        //     this._actionStatusText = text;
        //     PublishStatus();
        // }

        // public void PublishStatus() {
        //     if (_actionStatus == ActionStatus.NO_GOAL) {
        //         rosSocket.Publish(statusPublicationID, new GoalStatusArray());
        //     } else {
        //         rosSocket.Publish(statusPublicationID,
        //             new GoalStatusArray {
        //                 status_list = new GoalStatus[]
        //                 {
        //                     new GoalStatus {
        //                         status = (byte)_actionStatus,
        //                         text = _actionStatusText
        //                     }
        //                 }
        //             }
        //         );
        //     }
        // }

        protected void PublishFeedback<T>(T feedback) where T : ZOROSMessageInterface {
            ROSBridgeConnection.Publish<T>(feedback, ROSTopic + "/feedback");
            // action.action_feedback.status.status = (byte)_actionStatus;
            // action.action_feedback.status.goal_id = action.action_goal.goal_id;
            // rosSocket.Publish(feedbackPublicationID, action.action_feedback);
        }

        protected void PublishResult<T>(T result) where T : ZOROSMessageInterface {
            ROSBridgeConnection.Publish<T>(result, ROSTopic + "/result");
            // action.action_feedback.status.status = (byte)_actionStatus;
            // action.action_result.status.goal_id = action.action_goal.goal_id;
            // rosSocket.Publish(resultPublicationID, action.action_result);
        }
    }

}