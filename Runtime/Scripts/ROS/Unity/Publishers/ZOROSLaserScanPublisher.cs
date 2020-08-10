using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Sensor;
using ZO.ROS.MessageTypes.Geometry;
using ZO.ROS.Unity.Publisher;
using ZO.Sensors;

namespace ZO.ROS.Unity.Publisher {

    /// <summary>
    /// Publish /sensor/LaserScan message.
    /// </summary>
    [RequireComponent(typeof(ZOROSTransformPublisher))]
    [RequireComponent(typeof(ZOLIDAR2D))]
    public class ZOROSLaserScanPublisher : ZOROSUnityGameObjectBase {

        public ZOLIDAR2D _lidar2DSensor;

        /// <summary>
        /// The 2D LIDAR sensor to publish it's scan data.
        /// </summary>
        /// <value></value>
        public ZOLIDAR2D LIDAR2DSensor {
            get => _lidar2DSensor;
            set => _lidar2DSensor = value;
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

        private LaserScanMessage _rosLaserScanMessage = new LaserScanMessage();
        TransformStampedMessage _transformMessage = new TransformStampedMessage();

        protected override void ZOUpdateHzSynchronized() {
            if (ROSUnityManager) {
                // update transform
                //TransformStampedMessage tfStamped = new TransformStampedMessage();
                // _transformMessage.header.Update();
                // _transformMessage.header.frame_id = "map";
                // _transformMessage.child_frame_id = _lidar2DSensor.LidarID;
                // _transformMessage.UnityTransform = _lidar2DSensor.transform;

                // ROSUnityManager.BroadcastTransform(_transformMessage);
            }

        }

        protected override void ZOStart() {
            base.ZOStart();
            if (ZOROSBridgeConnection.Instance.IsConnected) {
                Initialize();
            }
        }

        private void Initialize() {
            // advertise
            ROSBridgeConnection.Advertise(ROSTopic, _rosLaserScanMessage.MessageType);

            // hookup to the sensor update delegate
            _lidar2DSensor.OnPublishDelegate = OnPublishLidarScanDelegate;

        }


        protected override void ZOOnDestroy() {
            ROSBridgeConnection?.UnAdvertise(ROSTopic);
        }

        public override void OnROSBridgeConnected(object rosUnityManager) {
            Debug.Log("INFO: ZOROSLaserScanPublisher::OnROSBridgeConnected");
            Initialize();
        }

        public override void OnROSBridgeDisconnected(object rosUnityManager) {
            Debug.Log("INFO: ZOROSLaserScanPublisher::OnROSBridgeDisconnected");
            ROSBridgeConnection.UnAdvertise(ROSTopic);
        }

        private Task OnPublishLidarScanDelegate(ZOLIDAR2D lidar, string name, float[] ranges) {
            _rosLaserScanMessage.header.Update();
            _rosLaserScanMessage.header.frame_id = TransformPublisher.ChildFrameID;
            _rosLaserScanMessage.angle_min = lidar.MinAngleDegrees * Mathf.Deg2Rad;
            _rosLaserScanMessage.angle_max = lidar.MaxAngleDegrees * Mathf.Deg2Rad;
            _rosLaserScanMessage.angle_increment = lidar.AngleIncrementDegrees * Mathf.Deg2Rad;
            _rosLaserScanMessage.time_increment = lidar.TimeIncrementSeconds;
            _rosLaserScanMessage.scan_time = lidar.ScanTimeSeconds;
            _rosLaserScanMessage.range_min = lidar.MinRangeDistanceMeters;
            _rosLaserScanMessage.range_max = lidar.MaxRangeDistanceMeters;
            _rosLaserScanMessage.ranges = ranges;

            ROSBridgeConnection.Publish(_rosLaserScanMessage, ROSTopic, Name);

            return Task.CompletedTask;
        }

        #region ZOSerializationInterface
        public override string Type {
            get { return "ros.publisher.scan"; }
        }


        public override JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("ros_topic", ROSTopic),
                new JProperty("update_rate_hz", UpdateRateHz),
                new JProperty("lidar2d_name", LIDAR2DSensor.Name)
            );
            JSON = json;
            return json;
        }

        public override void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            _json = json;
            Name = json["name"].Value<string>();
            ROSTopic = json["ros_topic"].Value<string>();
            UpdateRateHz = json["update_rate_hz"].Value<float>();

            // find connected 2d lidar.  needs to be done post load hence the Lamda
            documentRoot.OnPostDeserializationNotification((docRoot) => {
                if (JSON.ContainsKey("lidar2d_name")) {
                    ZOLIDAR2D[] lidars = docRoot.gameObject.GetComponentsInChildren<ZOLIDAR2D>();
                    foreach (ZOLIDAR2D l in lidars) {
                        if (l.Name == JSON["lidar2d_name"].Value<string>()) {
                            LIDAR2DSensor = l;
                        }
                    }
                }
            });

        }

        #endregion // ZOSerializationInterface

    }

}
