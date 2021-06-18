using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Sensor;
using ZO.Sensors;
using ZO.ROS.Unity;
using ZO.Document;

namespace ZO.ROS.Publisher {

    /// <summary>
    /// Publish left & right stereo camera over two ROS Image & ImageInfo topic messages.
    /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
    /// To test run: `rosrun image_view image_view image:=/unity_image/image _image_transport:=raw`
    /// </summary>
    public class ZOROSStereoImagePublisher : ZOROSUnityGameObjectBase {

        public ZORGBCamera _leftCameraSensor;
        public ZORGBCamera _rightCameraSensor;

        [Header("ROS Topics")]
        /// <summary>
        /// The ROS Image message topic to publish to.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        /// </summary>
        public string _leftImageROSTopic = "stereo/left/image_raw";
        public string _rightImageROSTopic = "stereo/right/image_raw";

        /// <summary>
        /// The CameraInfo message topic to publish to.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        /// </summary>
        public string _leftCameraInfoROSTopic = "stereo/left/camera_info";
        public string _rightCameraInfoROSTopic = "stereo/right/camera_info";


        /// <summary>
        /// The left camera sensor that we will publish.
        /// </summary>
        /// <value></value>
        public ZORGBCamera LeftCameraSensor {
            get => _leftCameraSensor;
            set => _leftCameraSensor = value;
        }

        /// <summary>
        /// The right camera sensor that we will publish.
        /// </summary>
        /// <value></value>
        public ZORGBCamera RightCameraSensor {
            get => _rightCameraSensor;
            set => _rightCameraSensor = value;
        }

        private ImageMessage _leftImageMessage = new ImageMessage();
        private ImageMessage _rightImageMessage = new ImageMessage();
        private CameraInfoMessage _leftCameraInfoMessage = new CameraInfoMessage();
        private CameraInfoMessage _rightCameraInfoMessage = new CameraInfoMessage();

        /// <summary>
        /// The ROS Image message topic to publish to.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        /// </summary>
        /// <value></value>
        public string LeftImageROSTopic {
            get => _leftImageROSTopic;
            set => _leftImageROSTopic = value;
        }

        /// <summary>
        /// The ROS Image message topic to publish to.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        /// </summary>
        /// <value></value>
        public string RightImageROSTopic {
            get => _rightImageROSTopic;
            set => _rightImageROSTopic = value;
        }

        /// <summary>
        /// The CameraInfo message topic to publish to.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        /// </summary>
        /// <value></value>
        public string LeftCameraInfoROSTopic {
            get => _leftCameraInfoROSTopic;
            set => _leftCameraInfoROSTopic = value;
        }

        /// <summary>
        /// The CameraInfo message topic to publish to.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        /// </summary>
        /// <value></value>
        public string RightCameraInfoROSTopic {
            get => _rightCameraInfoROSTopic;
            set => _rightCameraInfoROSTopic = value;
        }

        protected override void ZOStart() {
            base.ZOStart();
            if (ZOROSBridgeConnection.Instance.IsConnected) {
                Initialize();
            }
        }
        protected override void ZOOnDestroy() {
            ROSBridgeConnection?.UnAdvertise(ROSTopic);
        }


        private void Initialize() {
            // advertise
            ROSBridgeConnection.Advertise(LeftImageROSTopic, _leftImageMessage.MessageType);
            ROSBridgeConnection.Advertise(LeftCameraInfoROSTopic, _leftCameraInfoMessage.MessageType);
            ROSBridgeConnection.Advertise(RightImageROSTopic, _rightImageMessage.MessageType);
            ROSBridgeConnection.Advertise(RightCameraInfoROSTopic, _rightCameraInfoMessage.MessageType);

            // hookup to the sensor update delegate
            _leftCameraSensor.OnPublishRGBImageDelegate = OnPublishLeftRGBImageDelegate;
            _rightCameraSensor.OnPublishRGBImageDelegate = OnPublishRightRGBImageDelegate;

        }

        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeConnected");
            Initialize();

        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeDisconnected");
            ROSBridgeConnection?.UnAdvertise(LeftImageROSTopic);
            ROSBridgeConnection?.UnAdvertise(LeftCameraInfoROSTopic);
            ROSBridgeConnection?.UnAdvertise(RightImageROSTopic);
            ROSBridgeConnection?.UnAdvertise(RightCameraInfoROSTopic);

        }



        private int _cameraSync = 1;
        private bool IsCamerasSynched {
            get { return _cameraSync % 2 == 0; }
        }
        private void UpdateCameraSynchronization() {
            if (IsCamerasSynched == true) {
                // synchronize the headers
                _rightImageMessage.header = _leftImageMessage.header;
                _rightCameraInfoMessage.header = _leftImageMessage.header;
                _leftCameraInfoMessage.header = _leftImageMessage.header;

                // send the messages
                ROSBridgeConnection.Publish<ImageMessage>(_leftImageMessage, LeftImageROSTopic, LeftCameraSensor.Name);
                ROSBridgeConnection.Publish<CameraInfoMessage>(_leftCameraInfoMessage, LeftCameraInfoROSTopic, LeftCameraSensor.Name);
                ROSBridgeConnection.Publish<ImageMessage>(_rightImageMessage, RightImageROSTopic, RightCameraSensor.Name);
                ROSBridgeConnection.Publish<CameraInfoMessage>(_rightCameraInfoMessage, RightCameraInfoROSTopic, RightCameraSensor.Name);
            }
            _cameraSync++;
        }



        /// <summary>
        /// Publishes raw camera RBG8 data as a ROS Image message.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        /// </summary>
        /// <param name="rgbCamera">The camera component</param>
        /// <param name="cameraId">Camera ID</param>
        /// <param name="width">Frame width</param>
        /// <param name="height">Frame height</param>
        /// <param name="rgbData">Raw RBG8 data </param>
        /// <returns></returns>
        private Task OnPublishLeftRGBImageDelegate(ZORGBCamera rgbCamera, string cameraId, int width, int height, byte[] rgbData) {

            // figure out the transforms
            ZOROSTransformPublisher transformPublisher = GetComponent<ZOROSTransformPublisher>();
            if (transformPublisher != null) {
                _leftImageMessage.header.frame_id = transformPublisher.ChildFrameID;
                _leftCameraInfoMessage.header.frame_id = transformPublisher.ChildFrameID;
            } else {
                _leftImageMessage.header.frame_id = Name;
                _leftCameraInfoMessage.header.frame_id = Name;
            }

            // setup and send Image message
            _leftImageMessage.header.Update();
            _leftImageMessage.height = (uint)height;
            _leftImageMessage.width = (uint)width;
            _leftImageMessage.is_bigendian = 0;
            _leftImageMessage.data = rgbData;

            // if (LeftCameraSensor.IsMonochrome == true) {
            //     _leftImageMessage.step = 1 * (uint)width;
            //     _leftImageMessage.encoding = "mono8";
            // } else { // RGB
                _leftImageMessage.step = 1 * 3 * (uint)width;
                _leftImageMessage.encoding = "rgb8";
            // }

            // setup and send CameraInfo message            
            _leftCameraInfoMessage.Update();
            _leftCameraInfoMessage.header = _leftImageMessage.header;
            // initialize the camera info
            if (LeftCameraSensor.UnityCamera.usePhysicalProperties == true) {
                _leftCameraInfoMessage.BuildCameraInfo((uint)LeftCameraSensor.Width, (uint)LeftCameraSensor.Height,
                                                (double)LeftCameraSensor.FocalLengthMM,
                                                (double)LeftCameraSensor.SensorSizeMM.x, (double)LeftCameraSensor.SensorSizeMM.y);
            } else {
                _leftCameraInfoMessage.BuildCameraInfo((uint)LeftCameraSensor.Width, (uint)LeftCameraSensor.Height, (double)LeftCameraSensor.FieldOfViewDegrees);
            }

            UpdateCameraSynchronization();

            return Task.CompletedTask;
        }

        /// <summary>
        /// Publishes raw camera RBG8 data as a ROS Image message.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        /// </summary>
        /// <param name="rgbCamera">The camera component</param>
        /// <param name="cameraId">Camera ID</param>
        /// <param name="width">Frame width</param>
        /// <param name="height">Frame height</param>
        /// <param name="rgbData">Raw RBG8 data </param>
        /// <returns></returns>
        private Task OnPublishRightRGBImageDelegate(ZORGBCamera rgbCamera, string cameraId, int width, int height, byte[] rgbData) {

            // figure out the transforms
            ZOROSTransformPublisher transformPublisher = GetComponent<ZOROSTransformPublisher>();
            if (transformPublisher != null) {
                _rightImageMessage.header.frame_id = transformPublisher.ChildFrameID;
                _rightCameraInfoMessage.header.frame_id = transformPublisher.ChildFrameID;
            } else {
                _rightImageMessage.header.frame_id = Name;
                _rightCameraInfoMessage.header.frame_id = Name;
            }

            // setup and send Image message
            _rightImageMessage.header.Update();
            _rightImageMessage.height = (uint)height;
            _rightImageMessage.width = (uint)width;
            _rightImageMessage.is_bigendian = 0;
            _rightImageMessage.data = rgbData;

            // if (RightCameraSensor.IsMonochrome == true) {
            //     _rightImageMessage.step = (uint)width;
            //     _rightImageMessage.encoding = "mono8";
            // } else { // RGB
                _rightImageMessage.step = 1 * 3 * (uint)width;
                _rightImageMessage.encoding = "rgb8";
            // }

            // setup and send CameraInfo message            
            _rightCameraInfoMessage.Update();
            _rightCameraInfoMessage.header = _rightImageMessage.header;
            // initialize the camera info
            if (RightCameraSensor.UnityCamera.usePhysicalProperties == true) {
                _rightCameraInfoMessage.BuildCameraInfo((uint)RightCameraSensor.Width, (uint)RightCameraSensor.Height,
                                                (double)RightCameraSensor.FocalLengthMM,
                                                (double)RightCameraSensor.SensorSizeMM.x, (double)RightCameraSensor.SensorSizeMM.y);
            } else {
                _rightCameraInfoMessage.BuildCameraInfo((uint)RightCameraSensor.Width, (uint)RightCameraSensor.Height, (double)RightCameraSensor.FieldOfViewDegrees);
            }

            UpdateCameraSynchronization();

            return Task.CompletedTask;
        }

        #region ZOSerializationInterface
        public override string Type {
            get { return "ros.publisher.image"; }
        }



        #endregion // ZOSerializationInterface
    }

}
