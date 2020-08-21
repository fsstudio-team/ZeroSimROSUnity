using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Sensor;
using ZO.Sensors;
using ZO.ROS.Unity;

namespace ZO.ROS.Publisher {

    /// <summary>
    /// Publish /sensor/Image message.
    /// To test run: `rosrun image_view image_view image:=/unity_image/image _image_transport:=raw`
    /// </summary>
    public class ZOROSRGBDepthPublisher : ZOROSUnityGameObjectBase, ZOSerializationInterface {

        [Header("RGB Depth Publisher")]
        public ZORGBDepthCamera _rgbDepthCameraSensor;

        [Header("ROS Topics")]
        public string _rgbImageROSTopic = "rgb/image_rect_color";
        public string _depthROSTopic = "depth_registered/image_rect";
        public string _cameraInfoROSTopic = "rgb/camera_info";

        /// <summary>
        /// The depth camera sensor that we will publish.
        /// </summary>
        /// <value></value>
        public ZORGBDepthCamera RGBDepthCameraSensor {
            get => _rgbDepthCameraSensor;
            set => _rgbDepthCameraSensor = value;
        }
        private ImageMessage _rosColorImageMessage = new ImageMessage();
        private ImageMessage _rosDepthImageMessage = new ImageMessage();

        protected override void ZOReset() {
            base.ZOReset();
            UpdateRateHz = 30;
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
            ROSBridgeConnection.Advertise(ROSTopic, _rosColorImageMessage.MessageType);

            // hookup to the sensor update delegate
            _rgbDepthCameraSensor.OnPublishDelegate = OnPublishRGBDepthDelegate;

        }

        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeConnected");
            Initialize();

        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeDisconnected");
            ROSBridgeConnection.UnAdvertise(ROSTopic);
        }

        private Task OnPublishRGBDepthDelegate(ZORGBDepthCamera rgbdCamera, string cameraId, int width, int height, byte[] rgbData, float[] depthData) {
            _rosColorImageMessage.header.Update();
            _rosColorImageMessage.height = (uint)height;
            _rosColorImageMessage.width = (uint)width;
            _rosColorImageMessage.encoding = "rgb8";
            _rosColorImageMessage.is_bigendian = 0;
            _rosColorImageMessage.step = 1 * 3 * (uint)width;
            _rosColorImageMessage.data = rgbData;
            ROSBridgeConnection.Publish<ImageMessage>(_rosColorImageMessage, ROSTopic, Name);

            _rosDepthImageMessage.header.Update();
            _rosDepthImageMessage.height = (uint)height;
            _rosDepthImageMessage.width = (uint)width;
            _rosDepthImageMessage.encoding = "32FC1";
            _rosDepthImageMessage.is_bigendian = 0;
            _rosDepthImageMessage.step = 4 * (uint)width;
            _rosDepthImageMessage.data = new byte[4 * width * height];
            System.Buffer.BlockCopy(depthData, 0, _rosDepthImageMessage.data, 0, 4 * width * height);


            return Task.CompletedTask;
        }

        #region ZOSerializationInterface
        public override string Type {
            get { return "ros.publisher.image"; }
        }


        /// <summary>
        /// Serialize to ZoSim JSON.
        /// </summary>
        /// <param name="documentRoot"></param>
        /// <param name="parent"></param>
        /// <returns></returns>
        public override JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("ros_topic", ROSTopic),
                new JProperty("update_rate_hz", UpdateRateHz),
                new JProperty("rgbd_camera_name", RGBDepthCameraSensor.Name)
            );
            JSON = json;
            return json;
        }

        /// <summary>
        /// Deserialize from ZoSim JSON
        /// </summary>
        /// <param name="documentRoot"></param>
        /// <param name="json"></param>
        public override void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            _json = json;
            Name = json["name"].Value<string>();
            ROSTopic = json["ros_topic"].Value<string>();
            UpdateRateHz = json["update_rate_hz"].Value<float>();

            // find connected camera.  needs to be done post load hence the Lamda
            documentRoot.OnPostDeserializationNotification((docRoot) => {
                if (JSON.ContainsKey("rgbd_camera_name")) {
                    ZORGBDepthCamera[] rgbdCameras = docRoot.gameObject.GetComponentsInChildren<ZORGBDepthCamera>();
                    foreach (ZORGBDepthCamera camera in rgbdCameras) {
                        if (camera.Name == JSON["rgbd_camera_name"].Value<string>()) {
                            RGBDepthCameraSensor = camera;
                        }
                    }
                }
            });

        }

        #endregion // ZOSerializationInterface
    }

}
