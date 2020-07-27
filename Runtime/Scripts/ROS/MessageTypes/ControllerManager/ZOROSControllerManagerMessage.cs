using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.Geometry;

namespace ZO.ROS.MessageTypes.ControllerManager {
    public class HardwareInterfaceResourcesMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return HardwareInterfaceResourcesMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/HardwareInterfaceResources";

        /// <summary>
        /// Type of hardware interface.
        /// </summary>
        /// <value></value>
        public string hardware_interface { get; set; }

        /// <summary>
        /// List of resources belonging to the hardware interface
        /// </summary>
        /// <value></value>
        public string[] resources { get; set; }

        public HardwareInterfaceResourcesMessage() {
            this.hardware_interface = "";
            this.resources = new string[0];
        }

        public HardwareInterfaceResourcesMessage(string hardware_interface, string[] resources) {
            this.hardware_interface = hardware_interface;
            this.resources = resources;
        }

    }

    public class ControllerStateMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return ControllerStateMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/ControllerState";

        public string name { get; set; }
        public string state { get; set; }
        public string type { get; set; }
        public HardwareInterfaceResourcesMessage[] claimed_resources { get; set; }

        public ControllerStateMessage() {
            this.name = "";
            this.state = "";
            this.type = "";
            this.claimed_resources = new HardwareInterfaceResourcesMessage[0];
        }

        public ControllerStateMessage(string name, string state, string type, HardwareInterfaceResourcesMessage[] claimed_resources) {
            this.name = name;
            this.state = state;
            this.type = type;
            this.claimed_resources = claimed_resources;
        }
    }

    /// <summary>
    /// The ListControllers service returns a list of controller names/states/types of the
    /// controllers that are loaded inside the controller_manager.
    /// <see>http://docs.ros.org/api/controller_manager_msgs/html/srv/ListControllers.html</see>
    /// </summary>
    public class ListControllersResponse : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return ListControllersResponse.Type; } }

        // [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/ListControllers";


        public ControllerStateMessage[] controller { get; set; }

        public ListControllersResponse() {
            this.controller = new ControllerStateMessage[0];
        }

        public ListControllersResponse(ControllerStateMessage[] controller) {
            this.controller = controller;
        }
    }

}