using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using ZO.Util;
using ZO.Physics;
using ZO.Sensors;
using ZO.ROS;
using ZO.ROS.Publisher;
using ZO.ROS.Unity;
using ZO.ROS.MessageTypes;
using ZO.ROS.MessageTypes.Geometry;
using ZO.ROS.MessageTypes.StdSrvs;


namespace ZO.Controllers {

    [RequireComponent(typeof(ZOROSJointStatesPublisher))]
    [RequireComponent(typeof(ZOSpotCharacterController))]
    public class ZOSpotController : ZOROSUnityGameObjectBase {


        public string _TwistTopicSubscription = "/cmd_vel";
        private TwistMessage _twistMessage = new TwistMessage();
        private TwistWithCovarianceStampedMessage _twistPublishMessage = new TwistWithCovarianceStampedMessage();
        private ZOROSFakeOdometryPublisher _odomPublisher = null;
        public bool IsClaimed {
            get; set;
        } = false;

        public bool IsPowered {
            get; set;
        } = false;

        public bool IsStanding {
            get; set;
        } = false;





        protected override void ZOOnValidate() {
            base.ZOOnValidate();
            UpdateRateHz = 50;
        }

        protected override void ZOStart() {
            base.ZOStart();

            _odomPublisher = GetComponentInChildren<ZOROSFakeOdometryPublisher>(true);


        }



        protected override void ZOUpdate() {
            base.ZOUpdate();


        }

        protected override void ZOFixedUpdate() {
            base.ZOFixedUpdate();


        }

        protected override void ZOUpdateHzSynchronized() {
            base.ZOUpdateHzSynchronized();

            if (_odomPublisher && ROSBridgeConnection.IsConnected == true) {
                _twistPublishMessage.header = _odomPublisher.CurrentOdometryMessage.header;
                _twistPublishMessage.twist = _odomPublisher.CurrentOdometryMessage.twist;
                ROSBridgeConnection.Publish<TwistWithCovarianceStampedMessage>(_twistPublishMessage, "/odometry/twist");

            }
        }



        protected override void ZOOnGUI() {
            base.ZOOnGUI();
        }

        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZODifferentialDriveController::OnROSBridgeConnected");

            // subscribe to Twist Message (i.e. control)
            ROSBridgeConnection.Subscribe<TwistMessage>(Name, _TwistTopicSubscription, _twistMessage.MessageType, OnROSTwistMessageReceived);

            // advertise odometry twist
            ROSBridgeConnection.Advertise("/odometry/twist", TwistWithCovarianceStampedMessage.Type);

            // advertise services
            // claim robot
            ROSBridgeConnection.AdvertiseService<TriggerServiceRequest>("claim",
                TriggerServiceRequest.Type,
                (rosBridge, msg, id) => {
                    TriggerServiceRequest triggerServiceRequest = msg as TriggerServiceRequest;

                    Debug.Log("INFO: ZOSpotController::TriggerServiceRequest claim");

                    if (IsClaimed == true) {
                        ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Error: Already claimed"), "claim", false, id);

                    } else {
                        IsClaimed = true;
                        ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(true, "Success"), "claim", false, id);

                    }

                    return Task.CompletedTask;

                });

            // release robot claim
            ROSBridgeConnection.AdvertiseService<TriggerServiceRequest>("release",
                TriggerServiceRequest.Type,
                (rosBridge, msg, id) => {
                    TriggerServiceRequest triggerServiceRequest = msg as TriggerServiceRequest;

                    Debug.Log("INFO: ZOSpotController::TriggerServiceRequest release");

                    if (IsClaimed == false) {
                        ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Error: Not claimed"), "release", false, id);

                    } else {
                        IsClaimed = false;
                        ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(true, "Success"), "release", false, id);

                    }

                    return Task.CompletedTask;

                });

            // power on
            ROSBridgeConnection.AdvertiseService<TriggerServiceRequest>("power_on",
                TriggerServiceRequest.Type,
                (rosBridge, msg, id) => {
                    TriggerServiceRequest triggerServiceRequest = msg as TriggerServiceRequest;

                    Debug.Log("INFO: ZOSpotController::TriggerServiceRequest power_on");

                    if (IsPowered == true) {
                        ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Error: Already powered on"), "power_on", false, id);

                    } else {
                        if (IsClaimed == true) {
                            IsPowered = true;
                            ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(true, "Success"), "power_on", false, id);

                        } else { // can't power if not claimed
                            ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Error: Not claimed.  Run /claim service"), "power_on", false, id);

                        }

                    }

                    return Task.CompletedTask;

                });

            // power off
            ROSBridgeConnection.AdvertiseService<TriggerServiceRequest>("power_off",
                TriggerServiceRequest.Type,
                (rosBridge, msg, id) => {
                    TriggerServiceRequest triggerServiceRequest = msg as TriggerServiceRequest;

                    Debug.Log("INFO: ZOSpotController::TriggerServiceRequest power_off");

                    if (IsPowered == false) {
                        ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Error: Already powered off"), "power_off", false, id);

                    } else {
                        if (IsClaimed == true) {
                            IsPowered = false;
                            ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(true, "Success"), "power_off", false, id);

                        } else { // can't power if not claimed
                            ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Error: Not claimed.  Run /claim service"), "power_off", false, id);

                        }

                    }

                    return Task.CompletedTask;

                });

            // stand
            ROSBridgeConnection.AdvertiseService<TriggerServiceRequest>("stand",
                TriggerServiceRequest.Type,
                (rosBridge, msg, id) => {
                    TriggerServiceRequest triggerServiceRequest = msg as TriggerServiceRequest;

                    Debug.Log("INFO: ZOSpotController::TriggerServiceRequest stand");

                    if (IsStanding == true) {
                        ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Error: Already standing"), "stand", false, id);

                    } else {
                        if (IsClaimed == true) {
                            if (IsPowered == true) {
                                IsStanding = true;
                                ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(true, "Success"), "stand", false, id);

                            } else { // not powereed
                                ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Error: Not powered.  Need to power on"), "stand", false, id);


                            }

                        } else { // can't power if not claimed
                            ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Error: Not claimed.  Run /claim service"), "stand", false, id);

                        }

                    }

                    return Task.CompletedTask;

                });

            // power off
            ROSBridgeConnection.AdvertiseService<TriggerServiceRequest>("sit",
                TriggerServiceRequest.Type,
                (rosBridge, msg, id) => {
                    TriggerServiceRequest triggerServiceRequest = msg as TriggerServiceRequest;

                    Debug.Log("INFO: ZOSpotController::TriggerServiceRequest sit");

                    if (IsStanding == true) {
                        ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Error: Already standing"), "sit", false, id);

                    } else {
                        if (IsClaimed == true) {
                            if (IsPowered == true) {
                                IsStanding = false;
                                ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(true, "Success"), "sit", false, id);

                            } else { // not powereed
                                ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Error: Not powered.  Need to power on"), "sit", false, id);


                            }

                        } else { // can't power if not claimed
                            ROSBridgeConnection.ServiceResponse<TriggerServiceResponse>(new TriggerServiceResponse(false, "Not claimed.  Run /claim service"), "sit", false, id);

                        }

                    }

                    return Task.CompletedTask;

                });


        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            ROSBridgeConnection.UnAdvertise("/odometry/twist");
            ROSBridgeConnection.UnAdvertiseService("claim");
            ROSBridgeConnection.UnAdvertiseService("release");

            Debug.Log("INFO: ZODifferentialDriveController::OnROSBridgeDisconnected");
        }


        /// <summary>
        /// Handles subscribed to `TwistMessage` which controls the differential control steering and drive. 
        /// </summary>
        /// <param name="rosBridgeConnection">ROS Bridge Connection</param>
        /// <param name="msg">TwistMessage</param>
        /// <returns></returns>
        public Task OnROSTwistMessageReceived(ZOROSBridgeConnection rosBridgeConnection, ZOROSMessageInterface msg) {
            _twistMessage = (TwistMessage)msg;
            // Debug.Log("INFO: Twist Message Received: linear " + _twistMessage.linear.ToString() + " angular: " + _twistMessage.angular.ToString());

            return Task.CompletedTask;
        }

    }
}