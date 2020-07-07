using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.Geometry;

namespace ZO.ROS.MessageTypes.ROSAPI {

    /// <summary>
    /// Get rosparam parameter names via rosbridge
    /// See: rosapi/GetParamNames Service
    /// </summary>
    public class GetParamNamesResponse : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "rosapi/GetParamNames"; } }

        public string[] names { get; set; }
        public GetParamNamesResponse() {
            this.names = new string[0];
        }

        public GetParamNamesResponse(string[] names) {
            this.names = names;
        }

    }


    /// <summary>
    /// Get ROS param via ROS Bridge request.
    /// </summary>
    public class GetParamServiceRequest : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "rosapi/GetParam"; } }

        public string name { get; set; }
        public string @default { get; set; }
        public GetParamServiceRequest() {
            this.name = "";
            this.@default = "";
        }

        public GetParamServiceRequest(string name, string _default) {
            this.name = name;
            this.@default = _default;
        }

    }


    /// <summary>
    /// Get ROS param via ROS Bridge response.
    /// </summary>
    public class GetParamResponse : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "rosapi/GetParam"; } }

        public string value { get; set; }
        public GetParamResponse() {
            this.value = "";
        }

        public GetParamResponse(string value) {
            this.value = value;
        }
    }

    public class SetParamServiceRequest : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "rosapi/SetParam"; } }

        public string name { get; set; }
        public string value { get; set; }

        public SetParamServiceRequest() {
            this.name = "";
            this.value = "";
        }

        public SetParamServiceRequest(string name, string value) {
            this.name = name;
            this.value = value;
        }
    }


    /// <summary>
    /// Checks if ROS Parameter exists.
    /// See: http://docs.ros.org/hydro/api/rosapi/html/srv/HasParam.html
    /// </summary>
    public class HasParamServiceRequest : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "rosapi/HasParam"; } }

        public string name {get; set;}

        public HasParamServiceRequest() {
            this.name = "";
        }

        public HasParamServiceRequest(string name) {
            this.name = name;
        }
    }


    /// <summary>
    /// Response to existance of ROS parameter
    /// See: http://docs.ros.org/hydro/api/rosapi/html/srv/HasParam.html
    /// </summary>
    public class HasParamResponse : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "rosapi/HasParam"; } }

        public bool exists {get; set;}

        public HasParamResponse() {
            this.exists = false;
        }

        public HasParamResponse(bool exists) {
            this.exists = exists;
        }
        
    }
}