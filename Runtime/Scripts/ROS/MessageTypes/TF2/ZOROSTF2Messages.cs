
using ZO.ROS.MessageTypes.Std;

namespace ZO.ROS.MessageTypes.TF2 {

    public class TFMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "tf2_msgs/TFMessage"; } }

        public Geometry.TransformStampedMessage[] transforms;

        public TFMessage() {
            transforms = new Geometry.TransformStampedMessage[0];
        }
    }

}