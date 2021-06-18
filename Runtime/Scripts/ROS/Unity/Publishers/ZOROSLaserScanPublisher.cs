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
    /// Publish ROS /sensor/LaserScan message using the ZOLIDAR2D sensor.
    /// See: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
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

        public string _parentTransformId;


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


        protected override void ZOStart() {
            base.ZOStart();
            if (ZOROSBridgeConnection.Instance.IsConnected) {
                Initialize();
            }
        }

        protected override void ZOOnValidate() {
            base.ZOOnValidate();
            if (LIDAR2DSensor == null) {
                LIDAR2DSensor = GetComponent<ZOLIDAR2D>();                
            }

            if (ROSTopic == "") {
                ROSTopic = "scan";
            }

            if (UpdateRateHz == 0) {
                UpdateRateHz = 10;
            }

            if (_parentTransformId == "") {
                ZOROSTransformPublisher parentTransformPublisher = transform.parent.GetComponent<ZOROSTransformPublisher>();
                if (parentTransformPublisher != null) {
                    _parentTransformId = parentTransformPublisher.ChildFrameID;
                } else {
                    _parentTransformId = "base_footprint";
                }
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

        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOROSLaserScanPublisher::OnROSBridgeConnected");
            Initialize();
        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOROSLaserScanPublisher::OnROSBridgeDisconnected");
            ROSBridgeConnection.UnAdvertise(ROSTopic);
        }

        private Task OnPublishLidarScanDelegate(ZOLIDAR2D lidar, string name, float[] ranges) {
            _rosLaserScanMessage.header.Update();
            _rosLaserScanMessage.header.frame_id = _parentTransformId;
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



        #endregion // ZOSerializationInterface

    }

}
