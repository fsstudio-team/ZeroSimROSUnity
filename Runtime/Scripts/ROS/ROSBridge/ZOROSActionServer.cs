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
    public abstract class ZOROSActionServer<TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback> : ZOROSUnityGameObjectBase
        where TActionGoal : ZOROSMessageInterface, new()
        where TActionResult : ZOROSMessageInterface, new()
        where TActionFeedback : ZOROSMessageInterface, new() {

        // This is defined according to actionlib_msgs/GoalStatus
        public enum ActionStatus {
            NO_GOAL = -1,    // For internal server use. If status is NA, published status array will have length 0
            PENDING,    //  The goal has yet to be processed by the action server
            ACTIVE,     //  The goal is currently being processed by the action server
            PREEMPTED,  //  The goal received a cancel request after it started executing and has since completed its execution (Terminal State)
            SUCCEEDED,  //  The goal was achieved successfully by the action server (Terminal State)
            ABORTED,    //  The goal was aborted during execution by the action server due to some failure (Terminal State)
            REJECTED,   //  The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)
            PREEMPTING, //  The goal received a cancel request after it started executing and has not yet completed execution
            RECALLING,  //  The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled
            RECALLED,   //  The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)
            LOST,       //  An action client can determine that a goal is LOST. This should not be sent over the wire by an action server
        }
        private ActionStatus _actionStatus = ActionStatus.NO_GOAL;
        public ActionStatus Status {
            get => _actionStatus;

            protected set {
                _actionStatus = value;
                // publish status
                if (_actionStatus == ActionStatus.NO_GOAL) {
                    ROSBridgeConnection.Publish<GoalStatusArrayMessage>(new GoalStatusArrayMessage(), ROSTopic + "/status");
                } else {
                    ROSBridgeConnection.Publish<GoalStatusArrayMessage>(
                        new GoalStatusArrayMessage {
                            status_list = new GoalStatusMessage[]
                            {
                                new GoalStatusMessage {
                                    status = (byte)_actionStatus,
                                    text = _actionStatusText
                                }
                            }
                        },
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


        #region ZOROSUnityInterface
        public override void OnROSBridgeConnected(object rosUnityManager) {
            // Advertise Action feedback
            ROSBridgeConnection.Advertise(ROSTopic + "/status", GoalStatusArrayMessage.Type);
            ROSBridgeConnection.Advertise(ROSTopic + "/feedback", (new TActionFeedback()).MessageType);
            ROSBridgeConnection.Advertise(ROSTopic + "/result", (new TActionResult()).MessageType);

            // Subscribe to Action goal
            ROSBridgeConnection.Subscribe<GoalIDMessage>(Name, ROSTopic + "/cancel", GoalIDMessage.Type, OnCancelDelegate);
            ROSBridgeConnection.Subscribe<TActionGoal>(Name, ROSTopic + "/goal", (new TActionGoal()).MessageType, OnGoalDelegate);


            Status = ActionStatus.NO_GOAL;

        }

        public override void OnROSBridgeDisconnected(object rosUnityManager) {

            ROSBridgeConnection.UnAdvertise(ROSTopic + "/status");
            ROSBridgeConnection.UnAdvertise(ROSTopic + "/feedback");
            ROSBridgeConnection.UnAdvertise(ROSTopic + "/result");

            ROSBridgeConnection.Unsubscribe(ROSTopic + "/cancel");
            ROSBridgeConnection.Unsubscribe(ROSTopic + "/goal");

        }
        #endregion // ZOROSUnityInterface




        // Client Triggered Actions
        // When receive a new goal
        protected abstract void OnGoalReceived(ZOROSMessageInterface actionGoal);
        private Task OnGoalDelegate(ZOROSBridgeConnection oROSBridgeConnection, ZOROSMessageInterface msg) {

            Status = ActionStatus.PENDING;
            OnGoalReceived(msg);
            return Task.CompletedTask;
        }

        // When the goal is cancelled by the client
        protected abstract void OnGoalRecalling(GoalIDMessage goalID);
        protected abstract void OnGoalPreempting();
        private Task OnCancelDelegate(ZOROSBridgeConnection oROSBridgeConnection, ZOROSMessageInterface msg) {
            GoalIDMessage goalID = msg as GoalIDMessage;
            switch (Status) {
                case ActionStatus.PENDING:
                    Status  = ActionStatus.RECALLING;
                    OnGoalRecalling(goalID);
                    break;
                case ActionStatus.ACTIVE:
                    Status = ActionStatus.PREEMPTING;
                    OnGoalPreempting();
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot be canceled in current state: " + _actionStatus.ToString());
                    break;
            }

            return Task.CompletedTask;
        }

        // Server Triggered Actions
        protected abstract void OnGoalActive();
        protected void SetAccepted(string info = "") {
            ActionStatusText = info;
            switch (Status) {
                case ActionStatus.PENDING:
                    Status = ActionStatus.ACTIVE;
                    OnGoalActive();
                    break;
                case ActionStatus.RECALLING:
                    Status = ActionStatus.PREEMPTING;
                    OnGoalPreempting();
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot be accepted in current state: " + Status.ToString());
                    break;
            }
        }

        protected abstract void OnGoalRejected();
        protected void SetRejected(string reason = "") {
            ActionStatusText = reason;
            switch (Status) {
                case ActionStatus.PENDING:
                    Status = ActionStatus.REJECTED;
                    OnGoalRejected();
                    break;
                case ActionStatus.RECALLING:
                    Status = ActionStatus.REJECTED;
                    OnGoalRejected();
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot be rejected in current state: " + Status.ToString());
                    break;
            }
        }

        protected abstract void OnGoalSucceeded();
        protected void SetSucceeded(ZOROSMessageInterface result = null, string info = "") {
            ActionStatusText = info;
            switch (Status) {
                case ActionStatus.ACTIVE:
                    Status = ActionStatus.SUCCEEDED;
                    PublishResult(result);
                    OnGoalSucceeded();
                    break;
                case ActionStatus.PREEMPTING:
                    Status = ActionStatus.SUCCEEDED;
                    PublishResult(result);
                    OnGoalSucceeded();
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot succeed in current state: " + Status.ToString());
                    break;
            }
        }

        protected abstract void OnGoalAborted();
        protected void SetAborted(string reason = "") {
            ActionStatusText = reason;
            switch (Status) {
                case ActionStatus.ACTIVE:
                    Status = ActionStatus.ABORTED;
                    OnGoalAborted();
                    break;
                case ActionStatus.PREEMPTING:
                    Status = ActionStatus.ABORTED;
                    OnGoalAborted();
                    break;
                default:
                    Debug.LogWarning("WARNING Goal cannot be aborted in current state: " + Status.ToString());
                    break;
            }
        }

        protected abstract void OnGoalCanceled(ZOROSMessageInterface result);
        protected void SetCanceled(TResult result = null) {
            switch (Status) {
                case ActionStatus.RECALLING:
                    Status = ActionStatus.RECALLED;
                    break;
                case ActionStatus.PREEMPTING:
                    Status = ActionStatus.PREEMPTED;
                    break;
                default:
                    Debug.LogWarning("WARNING: Goal cannot be be canceled in current status state: " + Status.ToString());
                    return;
            }
            OnGoalCanceled(result);
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

        protected void PublishFeedback() {
            action.action_feedback.status.status = (byte)_actionStatus;
            action.action_feedback.status.goal_id = action.action_goal.goal_id;
            rosSocket.Publish(feedbackPublicationID, action.action_feedback);
        }

        protected void PublishResult(ZOROSMessageInterface result) {
            ROSBridgeConnection.Publish()
            action.action_feedback.status.status = (byte)_actionStatus;
            action.action_result.status.goal_id = action.action_goal.goal_id;
            rosSocket.Publish(resultPublicationID, action.action_result);
        }
    }

}