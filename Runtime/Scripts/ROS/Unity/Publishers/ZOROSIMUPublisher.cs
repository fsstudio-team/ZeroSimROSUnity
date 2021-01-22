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
            IMUSensor.OnPublishDelegate = OnPublishImuDelegate;

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

        private Task OnPublishImuDelegate(ZOIMU lidar, string name, Vector3 linearAccel, Vector3 angularVelocity, Quaternion orientation) {
            _imuMessage.header.Update();
            _imuMessage.header.frame_id = TransformPublisher.ChildFrameID;
            _imuMessage.linear_acceleration.UnityVector3 = linearAccel;
            _imuMessage.angular_velocity.UnityVector3 = angularVelocity;
            _imuMessage.orientation.UnityQuaternion = orientation;

            // if (_referenceFrame == ReferenceFrame.RightHanded_XBackward_YLeft_ZUp) {
            //     //  (x, y, z, w) -> (-x, -z, y, -w).
            //     publishedOrientation = new Quaternion(-transform.rotation.z, -transform.rotation.x, transform.rotation.y, -transform.rotation.w);
            //     // if (m_zReverse)
            //     //     rot = new Quaternion (-rot.x, rot.y, -rot.z, rot.w);     
            //     publishedAcceleration = new Vector3(-_acceleration.z, -_acceleration.x, _acceleration.y);
            //     publishedAngularVelocity = new Vector3(-_angularVelocity.z, -_angularVelocity.x, _angularVelocity.y);
            // }


            ROSBridgeConnection.Publish(_imuMessage, ROSTopic, Name);

            return Task.CompletedTask;
        }

        #region ZOSerializationInterface
        public override string Type {
            get { return "ros.publisher.imu"; }
        }


        public override JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("ros_topic", ROSTopic),
                new JProperty("update_rate_hz", UpdateRateHz),
                new JProperty("imu_name", IMUSensor.Name)
            );
            JSON = json;
            return json;
        }

        public override void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            _json = json;
            Name = json["name"].Value<string>();
            ROSTopic = json["ros_topic"].Value<string>();
            UpdateRateHz = json["update_rate_hz"].Value<float>();

            // find connected imu.  needs to be done post load hence the Lamda
            documentRoot.OnPostDeserializationNotification((docRoot) => {
                if (JSON.ContainsKey("imu_name")) {
                    ZOIMU[] imus = docRoot.gameObject.GetComponentsInChildren<ZOIMU>();
                    foreach (ZOIMU l in imus) {
                        if (l.Name == JSON["imu_name"].Value<string>()) {
                            IMUSensor = l;
                        }
                    }
                }
            });

        }

        #endregion // ZOSerializationInterface

    }

}
