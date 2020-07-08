using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.Geometry;

namespace ZO.ROS.MessageTypes.Gazebo {

    /// <summary>
    /// Spawn a model service request
    /// See: http://docs.ros.org/jade/api/gazebo_msgs/html/srv/SpawnModel.html
    /// </summary>
    public class SpawnModelServiceRequest : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "gazebo_msgs/SpawnModel"; } }


        /// <summary>
        /// name of the model to be spawn
        /// </summary>
        /// <value></value>
        public string model_name { get; set; }

        /// <summary>
        /// this should be an urdf or gazebo xml
        /// </summary>
        /// <value></value>
        public string model_xml { get; set; }

        /// <summary>
        /// spawn robot and all ROS interfaces under this namespace
        /// </summary>
        /// <value></value>            
        public string robot_namespace { get; set; }


        /// <summary>
        /// only applied to canonical body
        /// </summary>
        /// <value></value>
        public PoseMessage initial_pose { get; set; }


        /// <summary>
        /// initial_pose is defined relative to the frame of this model/body
        /// if left empty or "world", then gazebo world frame is used
        /// if non-existent model/body is specified, an error is returned
        /// and the model is not spawned
        /// </summary>
        /// <value></value>
        public string reference_frame { get; set; }


        public SpawnModelServiceRequest() {
            this.model_name = "";
            this.model_xml = "";
            this.initial_pose = new PoseMessage();
            this.reference_frame = "world";
        }

        public SpawnModelServiceRequest(string model_name, string model_xml, PoseMessage initial_pose, string reference_frame) {
            this.model_name = model_name;
            this.model_xml = model_xml;
            this.initial_pose = new PoseMessage();
            this.reference_frame = reference_frame;
        }
    }

        /// <summary>
    /// Spawn a model service request
    /// See: http://docs.ros.org/jade/api/gazebo_msgs/html/srv/SpawnModel.html
    /// </summary>
    public class SpawnModelServiceResponse : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "gazebo_msgs/SpawnModel"; } }

        public bool success {get; set;}
        public string status_message {get; set;}

        public SpawnModelServiceResponse() {
            this.success = true;
            this.status_message = "";
        }

        public SpawnModelServiceResponse(bool success, string status_message) {
            this.success = success;
            this.status_message = status_message;
        }
    }
}