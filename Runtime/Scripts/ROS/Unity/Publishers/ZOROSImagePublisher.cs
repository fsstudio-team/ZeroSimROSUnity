using System.IO;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.Sensor;
using ZO.Sensors;

namespace ZO.ROS.Unity.Publisher {

    /// <summary>
    /// Publish /sensor/Image message.
    /// To test run: `rosrun image_view image_view image:=/unity_image/image _image_transport:=raw`
    /// </summary>
    public class ZOROSImagePublisher : ZOROSUnityGameObjectBase {

        public ZORGBCamera _rgbCameraSensor;
        // public string _ROSTopic = "unity_image/image";
        // public string _ROSId = "camera";

        // public string ROSTopic { get => _ROSTopic; }
        // public string ROSId { get => _ROSId; }
        // private ZOROSBridgeConnection ROSBridgeConnection { get; set; }
        private ImageMessage _rosImageMessage = new ImageMessage();
        // private HeaderMessage _rosHeader = new HeaderMessage();


        private void OnDestroy() {
            ROSBridgeConnection?.UnAdvertise(_ROSTopic);
        }

        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager, ZOROSBridgeConnection rosBridgeConnection) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeConnected");
            // ROSBridgeConnection = rosBridgeConnection;

            // advertise
            rosBridgeConnection.Advertise(_ROSTopic, _rosImageMessage.MessageType, ROSId);

            // hookup to the sensor update delegate
            _rgbCameraSensor.OnPublishRGBImageDelegate = OnPublishRGBImageDelegate;
        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager, ZOROSBridgeConnection rosBridgeConnection) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeDisconnected");
        }

        private Task OnPublishRGBImageDelegate(ZORGBCamera rgbCamera, string cameraId, int width, int height, byte[] rgbData) {
            // _rosHeader.seq++;
            // rgbData = new byte[1]; //killme
            // _rosImageMessage = new ImageMessage(_rosHeader, (uint)height, (uint)width, "rgb8", 0, 1 * 3 * (uint)width, rgbData);
            // _rosHeader.Update();
            _rosImageMessage.header.Update();// = _rosHeader;
            _rosImageMessage.height = (uint)height;
            _rosImageMessage.width = (uint)width;
            _rosImageMessage.encoding = "rgb8";
            _rosImageMessage.is_bigendian = 0;
            _rosImageMessage.step = 1 * 3 * (uint)width;
            _rosImageMessage.data = rgbData;
            ROSBridgeConnection.Publish<ImageMessage>(_rosImageMessage, ROSTopic, ROSId);

            return Task.CompletedTask;
        }
    }

}
