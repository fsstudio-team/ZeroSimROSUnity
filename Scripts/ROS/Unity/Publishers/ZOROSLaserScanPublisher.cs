using System.Threading.Tasks;
using UnityEngine;
using Unity.Collections;
using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.Sensor;
using ZO.ROS.MessageTypes.Geometry;
using ZO.Sensors;

namespace ZO.ROS.Unity.Publisher {

    /// <summary>
    /// Publish /sensor/LaserScan message.
    /// To test run: `rosrun image_view image_view image:=/unity_image/image _image_transport:=raw`
    /// </summary>
    public class ZOROSLaserScanPublisher : ZOROSUnityGameObjectBase {

        public ZOLIDAR2D _lidar2DSensor;

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


        private void OnDestroy() {
            ROSBridgeConnection?.UnAdvertise(_ROSTopic);
        }

        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager, ZOROSBridgeConnection rosBridgeConnection) {
            Debug.Log("INFO: ZOROSLaserScanPublisher::OnROSBridgeConnected");

            // advertise
            rosBridgeConnection.Advertise(_ROSTopic, _rosLaserScanMessage.MessageType);

            // hookup to the sensor update delegate
            _lidar2DSensor.OnPublishDelegate = OnPublishLidarScanDelegate;
        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager, ZOROSBridgeConnection rosBridgeConnection) {
            Debug.Log("INFO: ZOROSLaserScanPublisher::OnROSBridgeDisconnected");
            ROSBridgeConnection.UnAdvertise(_ROSTopic);
        }

        private Task OnPublishLidarScanDelegate(ZOLIDAR2D lidar, string lidarId, float[] ranges) {
            _rosLaserScanMessage.header.Update();
            _rosLaserScanMessage.header.frame_id = lidar.LidarID;
            _rosLaserScanMessage.angle_min = lidar.AngelMinDegrees * Mathf.Deg2Rad;
            _rosLaserScanMessage.angle_max = lidar.AngleMaxDegrees * Mathf.Deg2Rad;
            _rosLaserScanMessage.angle_increment = lidar.AngleIncrementDegrees * Mathf.Deg2Rad;
            _rosLaserScanMessage.time_increment = lidar.TimeIncrementSeconds;
            _rosLaserScanMessage.scan_time = lidar.ScanTimeSeconds;
            _rosLaserScanMessage.range_min = lidar.RangeMin;
            _rosLaserScanMessage.range_max = lidar.RangeMax;
            _rosLaserScanMessage.ranges = ranges;

            ROSBridgeConnection.Publish(_rosLaserScanMessage, _ROSTopic, _ROSId);

            return Task.CompletedTask;
        }
    }

}
