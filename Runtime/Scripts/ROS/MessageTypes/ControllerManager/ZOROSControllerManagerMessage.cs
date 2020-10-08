using System;
using ZO.ROS.MessageTypes.Std;

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

    /// <summary>
    /// Controller Manager message that lists all the resources which is usually just the joint names.
    /// </summary>
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

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/ListControllers";


        public ControllerStateMessage[] controller { get; set; }

        public ListControllersResponse() {
            this.controller = new ControllerStateMessage[0];
        }

        public ListControllersResponse(ControllerStateMessage[] controller) {
            this.controller = controller;
        }
    }


    /// <summary>
    /// The LoadController service allows you to load a single controller 
    /// inside controller_manager
    ///
    /// To load a controller, specify the "name" of the controller. 
    /// The return value "ok" indicates if the controller was successfully
    /// constructed and initialized or not.
    /// <see>http://docs.ros.org/api/controller_manager_msgs/html/srv/LoadController.html</see>
    /// </summary>
    public class LoadControllerServiceRequest : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return LoadControllerServiceRequest.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/LoadController";


        public string name { get; set; }

        public LoadControllerServiceRequest() {
            this.name = "";
        }

        public LoadControllerServiceRequest(string name) {
            this.name = name;
        }
    }

    /// <summary>
    /// The LoadController service allows you to load a single controller 
    /// inside controller_manager
    ///
    /// To load a controller, specify the "name" of the controller. 
    /// The return value "ok" indicates if the controller was successfully
    /// constructed and initialized or not.
    /// <see>http://docs.ros.org/api/controller_manager_msgs/html/srv/LoadController.html</see>
    /// </summary>
    public class LoadControllerServiceResponse : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return LoadControllerServiceResponse.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/LoadController";


        public bool ok { get; set; }

        public LoadControllerServiceResponse() {
            this.ok = true;
        }

        public LoadControllerServiceResponse(bool ok) {
            this.ok = ok;
        }
    }


    /// <summary>
    /// The SwitchController service allows you stop a number of controllers
    /// and start a number of controllers, all in one single timestep of the
    /// controller_manager control loop.
    ///
    /// To switch controllers, specify
    ///  * the list of controller names to start,
    ///  * the list of controller names to stop, and
    ///  * the strictness (BEST_EFFORT or STRICT)
    ///    * STRICT means that switching will fail if anything goes wrong (an invalid
    ///      controller name, a controller that failed to start, etc. )
    ///    * BEST_EFFORT means that even when something goes wrong with on controller,
    ///      the service will still try to start/stop the remaining controllers
    ///  * start the controllers as soon as their hardware dependencies are ready, will
    ///    wait for all interfaces to be ready otherwise
    ///  * the timeout in seconds before aborting pending controllers. Zero for infinite
    ///
    /// The return value "ok" indicates if the controllers were switched
    /// successfully or not.  The meaning of success depends on the
    /// specified strictness.
    /// 
    /// <see>http://docs.ros.org/api/controller_manager_msgs/html/srv/SwitchController.html</see>
    /// </summary>
    public class SwitchControllerServiceRequest : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return SwitchControllerServiceRequest.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/SwitchController";

        public const Int32 BEST_EFFORT = 1; //  The goal has yet to be processed by the action server
        public const Int32 STRICT = 2; //  The goal is currently being processed by the action server

        public string[] start_controllers { get; set; }
        public string[] stop_controllers { get; set; }

        public Int32 strictness { get; set; }
        public bool start_asap { get; set; }
        public double timeout { get; set; }

        public SwitchControllerServiceRequest() {
            this.start_controllers = new string[0];
            this.stop_controllers = new string[0];
            this.strictness = BEST_EFFORT;
            this.start_asap = true;
            this.timeout = 0;
        }

        public SwitchControllerServiceRequest(string[] start_controllers, string[] stop_controllers, Int32 strictness, bool start_asap, double timeout) {
            this.start_controllers = start_controllers;
            this.stop_controllers = stop_controllers;
            this.strictness = strictness;
            this.start_asap = start_asap;
            this.timeout = timeout;
        }
    }

    /// <summary>
    /// The SwitchController service allows you stop a number of controllers
    /// and start a number of controllers, all in one single timestep of the
    /// controller_manager control loop.
    ///
    /// To switch controllers, specify
    ///  * the list of controller names to start,
    ///  * the list of controller names to stop, and
    ///  * the strictness (BEST_EFFORT or STRICT)
    ///    * STRICT means that switching will fail if anything goes wrong (an invalid
    ///      controller name, a controller that failed to start, etc. )
    ///    * BEST_EFFORT means that even when something goes wrong with on controller,
    ///      the service will still try to start/stop the remaining controllers
    ///  * start the controllers as soon as their hardware dependencies are ready, will
    ///    wait for all interfaces to be ready otherwise
    ///  * the timeout in seconds before aborting pending controllers. Zero for infinite
    ///
    /// The return value "ok" indicates if the controllers were switched
    /// successfully or not.  The meaning of success depends on the
    /// specified strictness.
    /// 
    /// <see>http://docs.ros.org/api/controller_manager_msgs/html/srv/SwitchController.html</see>
    /// </summary>
    public class SwitchControllerServiceResponse : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return SwitchControllerServiceResponse.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/SwitchController";


        public bool ok { get; set; }

        public SwitchControllerServiceResponse() {
            this.ok = true;
        }

        public SwitchControllerServiceResponse(bool ok) {
            this.ok = ok;
        }
    }

    /// <summary>
    /// The UnloadController service allows you to unload a single controller 
    /// from controller_manager
    ///
    /// To unload a controller, specify the "name" of the controller. 
    /// The return value "ok" indicates if the controller was successfully
    ///  unloaded or not
    /// <see>http://docs.ros.org/api/controller_manager_msgs/html/srv/UnloadController.html</see>
    /// </summary>
    public class UnloadControllerServiceRequest : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return UnloadControllerServiceRequest.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/UnloadController";


        public string name { get; set; }

        public UnloadControllerServiceRequest() {
            this.name = "";
        }

        public UnloadControllerServiceRequest(string name) {
            this.name = name;
        }
    }

    /// <summary>
    /// The UnloadController service allows you to unload a single controller 
    /// from controller_manager
    ///
    /// To unload a controller, specify the "name" of the controller. 
    /// The return value "ok" indicates if the controller was successfully
    ///  unloaded or not
    /// <see>http://docs.ros.org/api/controller_manager_msgs/html/srv/UnloadController.html</see>
    /// </summary>
    public class UnloadControllerServiceResponse : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return UnloadControllerServiceResponse.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/UnloadController";

        public bool ok { get; set; }

        public UnloadControllerServiceResponse() {
            this.ok = true;
        }

        public UnloadControllerServiceResponse(bool ok) {
            this.ok = ok;
        }
    }

    /// <summary>
    /// The ReloadControllerLibraries service will reload all controllers that are available in
    /// the system as plugins
    ///
    /// 
    /// Reloading libraries only works if there are no controllers loaded. If there
    /// are still some controllers loaded, the reloading will fail.
    /// If this bool is set to true, all loaded controllers will get
    /// killed automatically, and the reloading can succeed.
    /// <see>http://docs.ros.org/api/controller_manager_msgs/html/srv/ReloadControllerLibraries.html</see>
    /// </summary>
    public class ReloadControllerLibrariesServiceRequest : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return ReloadControllerLibrariesServiceRequest.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/ReloadControllerLibraries";

        public bool force_kill { get; set; }

        public ReloadControllerLibrariesServiceRequest() {
            this.force_kill = true;
        }

        public ReloadControllerLibrariesServiceRequest(bool ok) {
            this.force_kill = ok;
        }
    }


    /// <summary>
    /// The ReloadControllerLibraries service will reload all controllers that are available in
    /// the system as plugins
    ///
    /// 
    /// Reloading libraries only works if there are no controllers loaded. If there
    /// are still some controllers loaded, the reloading will fail.
    /// If this bool is set to true, all loaded controllers will get
    /// killed automatically, and the reloading can succeed.
    /// <see>http://docs.ros.org/api/controller_manager_msgs/html/srv/ReloadControllerLibraries.html</see>
    /// </summary>
    public class ReloadControllerLibrariesServiceResponse : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return ReloadControllerLibrariesServiceResponse.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/ReloadControllerLibraries";

        public bool ok { get; set; }

        public ReloadControllerLibrariesServiceResponse() {
            this.ok = true;
        }

        public ReloadControllerLibrariesServiceResponse(bool ok) {
            this.ok = ok;
        }
    }


    /// <summary>
    /// The ListControllers service returns a list of controller types that are known
    /// to the controller manager plugin mechanism.
    /// 
    /// <see>http://docs.ros.org/api/controller_manager_msgs/html/srv/ListControllerTypes.html</see>
    /// </summary>
    public class ListControllerTypesServiceResponse : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return ListControllerTypesServiceResponse.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "controller_manager_msgs/ListControllerTypes";

        public string[] types { get; set; }
        public string[] base_classes { get; set; }

        public ListControllerTypesServiceResponse() {
            this.types = new string[0];
            this.base_classes = new string[0];
        }

        public ListControllerTypesServiceResponse(string[] types, string[] base_classes) {
            this.types = types;
            this.base_classes = base_classes;
        }
    }

}