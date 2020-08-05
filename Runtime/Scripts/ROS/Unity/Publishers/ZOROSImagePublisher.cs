using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Sensor;
using ZO.Sensors;

namespace ZO.ROS.Unity.Publisher {

    /// <summary>
    /// Publish /sensor/Image message.
    /// To test run: `rosrun image_view image_view image:=/unity_image/image _image_transport:=raw`
    /// </summary>
    public class ZOROSImagePublisher : ZOROSUnityGameObjectBase, ZOSerializationInterface {

        public ZORGBCamera _rgbCameraSensor;

        /// <summary>
        /// The camera sensor that we will publish.
        /// </summary>
        /// <value></value>
        public ZORGBCamera RGBCameraSensor {
            get => _rgbCameraSensor;
            set => _rgbCameraSensor = value;
        }
        private ImageMessage _rosImageMessage = new ImageMessage();


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
            ROSBridgeConnection.Advertise(ROSTopic, _rosImageMessage.MessageType);

            // hookup to the sensor update delegate
            _rgbCameraSensor.OnPublishRGBImageDelegate = OnPublishRGBImageDelegate;

        }

        public override void OnROSBridgeConnected(object rosUnityManager) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeConnected");
            Initialize();

        }

        public override void OnROSBridgeDisconnected(object rosUnityManager) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeDisconnected");
            ROSBridgeConnection.UnAdvertise(ROSTopic);
        }

        private Task OnPublishRGBImageDelegate(ZORGBCamera rgbCamera, string cameraId, int width, int height, byte[] rgbData) {
            _rosImageMessage.header.Update();
            _rosImageMessage.height = (uint)height;
            _rosImageMessage.width = (uint)width;
            _rosImageMessage.encoding = "rgb8";
            _rosImageMessage.is_bigendian = 0;
            _rosImageMessage.step = 1 * 3 * (uint)width;
            _rosImageMessage.data = rgbData;
            ROSBridgeConnection.Publish<ImageMessage>(_rosImageMessage, ROSTopic, Name);

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
