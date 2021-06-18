using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Sensor;
using ZO.ROS.MessageTypes.Geometry;
using ZO.ROS.Publisher;
using ZO.Sensors;
using ZO.ROS.Unity;
using ZO.Document;

namespace ZO.ROS.Publisher {

    /// <summary>
    /// Publish ROS Imu message using ZOIMU sensor.
    /// </summary>
    [RequireComponent(typeof(ZOROSTransformPublisher))]
    [RequireComponent(typeof(ZOIMU))]
    public class ZOROSIMUPublisher : ZOROSUnityGameObjectBase {

        public ZOIMU _imuSensor;

        /// <summary>
        /// The IMU sensor.
        /// </summary>
        /// <value></value>
        public ZOIMU IMUSensor {
            get => _imuSensor;
            set => _imuSensor = value;
        }

        public enum CoordinateSystemEnum {
            RightHanded_XBackward_YLeft_ZUp,
            RightHanded_XRight_YDown_ZForward,       // RealSense D435i
            Unity_LeftHanded_XRight_YUp_ZForward,   // Unity Standard
            ROS_RightHanded_XForward_YLeft_ZUp,                 // ROS Standard
        };

        public CoordinateSystemEnum _coordinateSystem = CoordinateSystemEnum.ROS_RightHanded_XForward_YLeft_ZUp;
        /// <summary>
        /// The coordinate system for the published IMU data.
        /// RightHanded_XBackward_YLeft_ZUp:  ? Some IMU's want this reference frame
        /// RightHanded_XRight_YDown_ZForward: RealSense D435i IMU prefers this reference frame.
        /// Unity_LeftHanded_XRight_YUp_ZForward:  Unity reference frame
        /// ROS_RightHanded_XForward_YLeft_ZUp: ROS standard reference frame.
        /// </summary>
        /// <value></value>
        public CoordinateSystemEnum CoordinateSystem {
            get { return _coordinateSystem; }
            set { _coordinateSystem = value; }
        }

        public bool _reverseLinearAccelerations = false;

        /// <summary>
        /// Flips gravity vector. 
        /// Why would you: the accelerometer is an intertial sensor and it measures inertial force. 
        /// The accelerometer doesn’t measure G-force, but rather the force that resists to G. 
        /// The resisting force aims up to the ceiling.
        /// </summary>
        /// <value></value>
        public bool ReverseLinearAccelerations {
            get { return _reverseLinearAccelerations; }
            set { _reverseLinearAccelerations = value; }
        }



        private ZOROSTransformPublisher _transformPublisher = null;
        public ZOROSTransformPublisher TransformPublisher {
            get {
                if (_transformPublisher == null) {
                    _transformPublisher = GetComponent<ZOROSTransformPublisher>();
                }

                if (_transformPublisher == null) {
                    Debug.LogError("ERROR: ZOROSLaserScanPublisher is missing corresponding ZOROSTransformPublisher");
                }
                return _transformPublisher;
            }
        }

        private ImuMessage _imuMessage = new ImuMessage();


        protected override void ZOStart() {
            base.ZOStart();
            if (ZOROSBridgeConnection.Instance.IsConnected) {
                Initialize();
            }
        }

        private void Initialize() {
            // advertise
            ROSBridgeConnection.Advertise(ROSTopic, _imuMessage.MessageType);

            // hookup to the sensor update delegate
            IMUSensor.OnPublishDelegate += OnPublishImuDelegate;

        }


        protected override void ZOOnDestroy() {
            ROSBridgeConnection?.UnAdvertise(ROSTopic);
        }

        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOROSLaserScanPublisher::OnROSBridgeConnected");
            Initialize();
        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOROSLaserScanPublisher::OnROSBridgeDisconnected");
            ROSBridgeConnection.UnAdvertise(ROSTopic);
        }

        private Task OnPublishImuDelegate(ZOIMU lidar, string name, Vector3 linearAccel, Vector3 gravity, Vector3 angularVelocity, Quaternion orientation) {
            _imuMessage.header.Update();
            _imuMessage.header.frame_id = TransformPublisher.ChildFrameID;


            if (ReverseLinearAccelerations == true) {
                linearAccel = linearAccel * -1;
            }


            if (_coordinateSystem == CoordinateSystemEnum.ROS_RightHanded_XForward_YLeft_ZUp) {
                _imuMessage.linear_acceleration.UnityVector3 = (linearAccel + gravity); 
                _imuMessage.angular_velocity.UnityVector3 = angularVelocity;
                _imuMessage.orientation.UnityQuaternion = orientation;

            } else if (CoordinateSystem == CoordinateSystemEnum.RightHanded_XBackward_YLeft_ZUp) {
                //  (x, y, z, w) -> (-x, -z, y, -w).
                _imuMessage.orientation.x = -orientation.z;
                _imuMessage.orientation.y = -orientation.x;
                _imuMessage.orientation.z = orientation.y;
                _imuMessage.orientation.w = -orientation.w;

                _imuMessage.linear_acceleration.x = -linearAccel.z + -gravity.z;
                _imuMessage.linear_acceleration.y = -linearAccel.x + -gravity.x;
                _imuMessage.linear_acceleration.z = linearAccel.y + gravity.y;

                _imuMessage.angular_velocity.x = -angularVelocity.z;
                _imuMessage.angular_velocity.y = -angularVelocity.x;
                _imuMessage.angular_velocity.z = angularVelocity.y;
            } else if (CoordinateSystem == CoordinateSystemEnum.RightHanded_XRight_YDown_ZForward) { // aka RealSense
                Quaternion flippedYOrientation = Quaternion.Euler(Vector3.Scale(orientation.eulerAngles, Vector3.up * -1));
                _imuMessage.orientation.x = flippedYOrientation.x;
                _imuMessage.orientation.y = flippedYOrientation.y;
                _imuMessage.orientation.z = flippedYOrientation.z;
                _imuMessage.orientation.w = -flippedYOrientation.w;  // negate left to right handed coordinate system

                _imuMessage.linear_acceleration.x = linearAccel.x + gravity.x;
                _imuMessage.linear_acceleration.y = -linearAccel.y + gravity.y;
                _imuMessage.linear_acceleration.z = linearAccel.z + gravity.z;

                _imuMessage.angular_velocity.x = -angularVelocity.x; // pitch
                _imuMessage.angular_velocity.y = angularVelocity.y; // yaw
                _imuMessage.angular_velocity.z = -angularVelocity.z; // roll

            } else if (CoordinateSystem == CoordinateSystemEnum.Unity_LeftHanded_XRight_YUp_ZForward) {
                _imuMessage.orientation.x = orientation.x;
                _imuMessage.orientation.y = orientation.y;
                _imuMessage.orientation.z = orientation.z;
                _imuMessage.orientation.w = orientation.w;

                _imuMessage.linear_acceleration.x = linearAccel.x + gravity.x;
                _imuMessage.linear_acceleration.y = linearAccel.y + gravity.y;
                _imuMessage.linear_acceleration.z = linearAccel.z + gravity.z;

                _imuMessage.angular_velocity.x = angularVelocity.x;
                _imuMessage.angular_velocity.y = angularVelocity.y;
                _imuMessage.angular_velocity.z = angularVelocity.z;

            }




            ROSBridgeConnection.Publish(_imuMessage, ROSTopic, Name);

            return Task.CompletedTask;
        }

        #region ZOSerializationInterface
        public override string Type {
            get { return "ros.publisher.imu"; }
        }



        #endregion // ZOSerializationInterface

    }

}
