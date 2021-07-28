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
using ZO.ROS.MessageTypes.Nav;


namespace ZO.Controllers {

    [RequireComponent(typeof(ZOROSJointStatesPublisher))]
    [RequireComponent(typeof(ZOSpotCharacterController))]
    public class ZOSpotController : ZOROSUnityGameObjectBase {


        public string _TwistTopicSubscription = "/cmd_vel";
        private TwistMessage _twistMessage = new TwistMessage();
        private TwistWithCovarianceStampedMessage _twistPublishMessage = new TwistWithCovarianceStampedMessage();
        private ZOROSFakeOdometryPublisher _odomPublisher = null;


        


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

            if (_odomPublisher && ZOROSBridgeConnection.Instance.IsConnected == true) {
                _twistPublishMessage.header = _odomPublisher.CurrentOdometryMessage.header;            
                _twistPublishMessage.twist = _odomPublisher.CurrentOdometryMessage.twist;
                ZOROSBridgeConnection.Instance.Publish<TwistWithCovarianceStampedMessage>(_twistPublishMessage, "/odometry/twist");
                
            }
        }



        protected override void ZOOnGUI() {
            base.ZOOnGUI();
        }

        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZODifferentialDriveController::OnROSBridgeConnected");

            // subscribe to Twist Message
            ZOROSBridgeConnection.Instance.Subscribe<TwistMessage>(Name, _TwistTopicSubscription, _twistMessage.MessageType, OnROSTwistMessageReceived);
            
            // advertise twist
            ZOROSBridgeConnection.Instance.Advertise("/odometry/twist", TwistWithCovarianceStampedMessage.Type);

        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            ZOROSBridgeConnection.Instance.UnAdvertise("/odometry/twist");
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