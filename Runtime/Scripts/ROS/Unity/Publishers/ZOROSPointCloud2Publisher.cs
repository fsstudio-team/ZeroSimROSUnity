using System.Threading.Tasks;
using UnityEngine;
using Unity.Collections;
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
    [RequireComponent(typeof(ZOLIDAR3D))]
    public class ZOROSPointCloud2Publisher : ZOROSUnityGameObjectBase {

        public ZOLIDAR3D _lidar3DSensor;

        /// <summary>
        /// The 2D LIDAR sensor to publish it's scan data.
        /// </summary>
        /// <value></value>
        public ZOLIDAR3D LIDAR3DSensor {
            get => _lidar3DSensor;
            set => _lidar3DSensor = value;
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

        private PointCloud2Message _pointCloud2Message = new PointCloud2Message();


        protected override void ZOStart() {
            base.ZOStart();
            if (ZOROSBridgeConnection.Instance.IsConnected) {
                Initialize();
            }
        }

        protected override void ZOOnValidate() {
            base.ZOOnValidate();
            if (LIDAR3DSensor == null) {
                LIDAR3DSensor = GetComponent<ZOLIDAR3D>();
            }

            if (ROSTopic == "") {
                ROSTopic = "point_cloud";
            }

            if (UpdateRateHz == 0) {
                UpdateRateHz = 10;
            }

            if (_parentTransformId == "" || _parentTransformId == null) {
                if (transform.parent != null) {
                    ZOROSTransformPublisher parentTransformPublisher = transform.parent.GetComponent<ZOROSTransformPublisher>();
                    if (parentTransformPublisher != null) {
                        _parentTransformId = parentTransformPublisher.ChildFrameID;
                    } else {
                        _parentTransformId = ZOROSUnityManager.Instance.WorldFrameId;
                    }

                } else {
                    _parentTransformId = ZOROSUnityManager.Instance.WorldFrameId;
                }
            }
        }

        private void Initialize() {
            // advertise
            ROSBridgeConnection.Advertise(ROSTopic, _pointCloud2Message.MessageType);

            // hookup to the sensor update delegate
            _lidar3DSensor.OnPublishDelegate = OnPublishLidarScanDelegate;

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

        private Task OnPublishLidarScanDelegate(ZOLIDAR3D lidar, string name, NativeArray<Vector3> positions, NativeArray<Vector3> normals) {
            _pointCloud2Message = PointCloud2Message.CreateXYZPointCloud(positions.ToArray());
            _pointCloud2Message.header.Update();
            _pointCloud2Message.header.frame_id = _parentTransformId;


            ROSBridgeConnection.Publish(_pointCloud2Message, ROSTopic, Name);

            return Task.CompletedTask;
        }

        #region ZOSerializationInterface
        public override string Type {
            get { return "ros.publisher.point_cloud2"; }
        }



        #endregion // ZOSerializationInterface

    }

}
