namespace ZO.ROS.MessageTypes.StdSrvs {

    /// <summary>
    /// See: http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html
    /// </summary>
    public class TriggerServiceRequest : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return TriggerServiceRequest.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "std_srvs/Trigger";


        public TriggerServiceRequest() {
        }

    }

    /// <summary>
    /// See: http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html
    /// </summary>
    public class TriggerServiceResponse : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return TriggerServiceResponse.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "std_srvs/Trigger";


        /// <summary>
        /// Indicate successful run of triggered service
        /// </summary>
        /// <value></value>
        public bool success { get; set; }

        /// <summary>
        /// Informational, e.g. for error messages
        /// </summary>
        /// <value></value>
        public string message { get; set; }

        public TriggerServiceResponse() {
            this.success = true;
            this.message = "";
        }

        public TriggerServiceResponse(bool success, string message) {
            this.success = success;
            this.message = message;
        }
    }


}