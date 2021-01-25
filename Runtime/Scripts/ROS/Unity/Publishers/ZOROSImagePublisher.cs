using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Sensor;
using ZO.Sensors;
using ZO.ROS.Unity;
using ZO.Document;

namespace ZO.ROS.Publisher {

    /// <summary>
    /// Publish /sensor/Image message.
    /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
    /// To test run: `rosrun image_view image_view image:=/unity_image/image _image_transport:=raw`
    /// </summary>
    public class ZOROSImagePublisher : ZOROSUnityGameObjectBase, ZOSerializationInterface {

        public ZORGBCamera _rgbCameraSensor;

        [Header("ROS Topics")]
        /// <summary>
        /// The ROS Image message topic to publish to.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        /// </summary>
        public string _imageROSTopic = "image/image_raw";

        /// <summary>
        /// The CameraInfo message topic to publish to.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        /// </summary>
        public string _cameraInfoROSTopic = "image/camera_info";


        /// <summary>
        /// The camera sensor that we will publish.
        /// </summary>
        /// <value></value>
        public ZORGBCamera RGBCameraSensor {
            get => _rgbCameraSensor;
            set => _rgbCameraSensor = value;
        }

        private ImageMessage _rosImageMessage = new ImageMessage();
        private CameraInfoMessage _rosCameraInfoMessage = new CameraInfoMessage();

        /// <summary>
        /// The ROS Image message topic to publish to.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        /// </summary>
        /// <value></value>
        public string ImageROSTopic {
            get => _imageROSTopic;
            set => _imageROSTopic = value;
        }

        /// <summary>
        /// The CameraInfo message topic to publish to.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        /// </summary>
        /// <value></value>
        public string CameraInfoROSTopic {
            get => _cameraInfoROSTopic;
            set => _cameraInfoROSTopic = value;
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
            ROSBridgeConnection.Advertise(ImageROSTopic, _rosImageMessage.MessageType);
            ROSBridgeConnection.Advertise(CameraInfoROSTopic, _rosCameraInfoMessage.MessageType);


            // hookup to the sensor update delegate
            if (RGBCameraSensor.IsMonochrome == true) {
                _rgbCameraSensor.OnPublishRGBImageDelegate = OnPublishMonoImageDelegate;
            } else {
                _rgbCameraSensor.OnPublishRGBImageDelegate = OnPublishRGBImageDelegate;
            }
            

        }

        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeConnected");
            Initialize();

        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeDisconnected");
            ROSBridgeConnection?.UnAdvertise(ImageROSTopic);
            ROSBridgeConnection?.UnAdvertise(CameraInfoROSTopic);
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
        private Task OnPublishRGBImageDelegate(ZORGBCamera rgbCamera, string cameraId, int width, int height, byte[] rgbData) {

            // figure out the transforms
            ZOROSTransformPublisher transformPublisher = GetComponent<ZOROSTransformPublisher>();
            if (transformPublisher != null) {
                _rosImageMessage.header.frame_id = transformPublisher.ChildFrameID;
                _rosCameraInfoMessage.header.frame_id = transformPublisher.ChildFrameID;
            } else {
                _rosImageMessage.header.frame_id = Name;
                _rosCameraInfoMessage.header.frame_id = Name;
            }

            // setup and send Image message
            _rosImageMessage.header.Update();
            _rosImageMessage.height = (uint)height;
            _rosImageMessage.width = (uint)width;
            _rosImageMessage.encoding = "rgb8";
            _rosImageMessage.is_bigendian = 0;
            _rosImageMessage.step = 1 * 3 * (uint)width;
            _rosImageMessage.data = rgbData;
            ROSBridgeConnection.Publish<ImageMessage>(_rosImageMessage, _imageROSTopic, Name);

            // setup and send CameraInfo message            
            _rosCameraInfoMessage.Update();
            _rosCameraInfoMessage.header = _rosImageMessage.header;
            // initialize the camera info
            if (RGBCameraSensor.UnityCamera.usePhysicalProperties == true) {
                _rosCameraInfoMessage.BuildCameraInfo((uint)RGBCameraSensor.Width, (uint)RGBCameraSensor.Height,
                                                (double)RGBCameraSensor.FocalLengthMM,
                                                (double)RGBCameraSensor.SensorSizeMM.x, (double)RGBCameraSensor.SensorSizeMM.y);
            } else {
                _rosCameraInfoMessage.BuildCameraInfo((uint)RGBCameraSensor.Width, (uint)RGBCameraSensor.Height, (double)RGBCameraSensor.FieldOfViewDegrees);
            }

            ROSBridgeConnection.Publish<CameraInfoMessage>(_rosCameraInfoMessage, _cameraInfoROSTopic, cameraId);

            return Task.CompletedTask;
        }

        /// <summary>
        /// Publishes raw camera U8 data as a ROS Image message.
        /// See: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        /// </summary>
        /// <param name="rgbCamera">The camera component</param>
        /// <param name="cameraId">Camera ID</param>
        /// <param name="width">Frame width</param>
        /// <param name="height">Frame height</param>
        /// <param name="rgbData">Raw RBG8 data. Note: we only sample the first channel assuming all channels have same monochrome value. </param>
        /// <returns></returns>
        private Task OnPublishMonoImageDelegate(ZORGBCamera rgbCamera, string cameraId, int width, int height, byte[] rgbData) {

            // figure out the transforms
            ZOROSTransformPublisher transformPublisher = GetComponent<ZOROSTransformPublisher>();
            if (transformPublisher != null) {
                _rosImageMessage.header.frame_id = transformPublisher.ChildFrameID;
                _rosCameraInfoMessage.header.frame_id = transformPublisher.ChildFrameID;
            } else {
                _rosImageMessage.header.frame_id = Name;
                _rosCameraInfoMessage.header.frame_id = Name;
            }

            // setup and send Image message
            _rosImageMessage.header.Update();
            _rosImageMessage.height = (uint)height;
            _rosImageMessage.width = (uint)width;
            _rosImageMessage.encoding = "mono8";
            _rosImageMessage.is_bigendian = 0;
            _rosImageMessage.step = 1 * (uint)width;
            _rosImageMessage.data = rgbData;
            ROSBridgeConnection.Publish<ImageMessage>(_rosImageMessage, _imageROSTopic, Name);

            // setup and send CameraInfo message            
            _rosCameraInfoMessage.Update();
            _rosCameraInfoMessage.header = _rosImageMessage.header;
            // initialize the camera info
            if (RGBCameraSensor.UnityCamera.usePhysicalProperties == true) {
                _rosCameraInfoMessage.BuildCameraInfo((uint)RGBCameraSensor.Width, (uint)RGBCameraSensor.Height,
                                                (double)RGBCameraSensor.FocalLengthMM,
                                                (double)RGBCameraSensor.SensorSizeMM.x, (double)RGBCameraSensor.SensorSizeMM.y);
            } else {
                _rosCameraInfoMessage.BuildCameraInfo((uint)RGBCameraSensor.Width, (uint)RGBCameraSensor.Height, (double)RGBCameraSensor.FieldOfViewDegrees);
            }

            ROSBridgeConnection.Publish<CameraInfoMessage>(_rosCameraInfoMessage, _cameraInfoROSTopic, cameraId);

            return Task.CompletedTask;
        }

        #region ZOSerializationInterface
        public override string Type {
            get { return "ros.publisher.image"; }
        }


        public override JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("ros_topic", ROSTopic),
                new JProperty("update_rate_hz", UpdateRateHz),
                new JProperty("rgb_camera_name", RGBCameraSensor.Name)
            );
            JSON = json;
            return json;
        }

        public override void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            _json = json;
            Name = json["name"].Value<string>();
            ROSTopic = json["ros_topic"].Value<string>();
            UpdateRateHz = json["update_rate_hz"].Value<float>();

            // find connected camera.  needs to be done post load hence the Lamda
            documentRoot.OnPostDeserializationNotification((docRoot) => {
                if (JSON.ContainsKey("rgb_camera_name")) {
                    ZORGBCamera[] rgbCameras = docRoot.gameObject.GetComponentsInChildren<ZORGBCamera>();
                    foreach (ZORGBCamera camera in rgbCameras) {
                        if (camera.Name == JSON["rgb_camera_name"].Value<string>()) {
                            RGBCameraSensor = camera;
                        }
                    }
                }
            });

        }

        #endregion // ZOSerializationInterface
    }

}
