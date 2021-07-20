using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.Geometry;

namespace ZO.ROS.MessageTypes.Nav {

    /// <summary>
    /// This represents an estimate of a position and velocity in free space.  
    /// The pose in this message should be specified in the coordinate frame given by header.frame_id.
    /// The twist in this message should be specified in the coordinate frame given by the child_frame_id
    /// See: http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
    /// </summary>
    public class OdometryMessage : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return OdometryMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "nav_msgs/Odometry";


        public HeaderMessage header { get; set; }
        public string child_frame_id { get; set; }

        public PoseWithCovarianceMessage pose { get; set; }
        public TwistWithCovarianceMessage twist { get; set; }

        public OdometryMessage() {
            this.header = new HeaderMessage();
            this.child_frame_id = "";
            this.pose = new PoseWithCovarianceMessage();
            this.twist = new TwistWithCovarianceMessage();
        }

        public OdometryMessage(HeaderMessage header, string child_frame_id, PoseWithCovarianceMessage pose, TwistWithCovarianceMessage twist) {
            this.header = header;
            this.child_frame_id = child_frame_id;
            this.pose = pose;
            this.twist = twist;
        }

        public void Update() {
             header.Update();
        }

    }
}