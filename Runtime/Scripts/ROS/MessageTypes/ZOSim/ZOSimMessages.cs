
using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.Geometry;

namespace ZO.ROS.MessageTypes.ZOSim {

    /// <summary>
    /// 
    /// Spawn a zerosim model service request
    /// <code>
    /// string model_name                 # name of the model to be spawn
    /// string model_zosim                # this should be an .zosim json file all in text
    /// string robot_namespace            # spawn robot and all ROS interfaces under this namespace
    /// geometry_msgs/Pose initial_pose   # only applied to canonical body
    /// string reference_frame            # initial_pose is defined relative to the frame of this model/body
    ///                                 # if left empty or "world", then zerosim world frame is used
    ///                                 # if non-existent model/body is specified, an error is returned
    ///                                 #   and the model is not spawned
    /// ---
    /// bool success                      # return true if spawn successful
    /// string status_message             # comments if available 
    /// </code>
    /// </summary>
    public class ZOSimSpawnServiceRequest : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "zero_sim_ros/ZOSimSpawn"; } }


        /// <summary>
        /// name of the model to be spawn
        /// </summary>
        /// <value></value>
        public string model_name { get; set; }

        /// <summary>
        /// this should be an zosim json string
        /// </summary>
        /// <value></value>
        public string model_zosim { get; set; }

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


        public ZOSimSpawnServiceRequest() {
            this.model_name = "";
            this.model_zosim = "";
            this.initial_pose = new PoseMessage();
            this.reference_frame = "world";
        }

        public ZOSimSpawnServiceRequest(string model_name, string model_xml, PoseMessage initial_pose, string reference_frame) {
            this.model_name = model_name;
            this.model_zosim = model_xml;
            this.initial_pose = new PoseMessage();
            this.reference_frame = reference_frame;
        }
    }

    /// <summary>
    /// Spawn a zerosim model service response
    /// </summary>
    public class ZOSimSpawnServiceResponse : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "zero_sim_ros/ZOSimSpawn"; } }

        public bool success { get; set; }
        public string status_message { get; set; }

        public ZOSimSpawnServiceResponse() {
            this.success = true;
            this.status_message = "";
        }

        public ZOSimSpawnServiceResponse(bool success, string status_message) {
            this.success = success;
            this.status_message = status_message;
        }
    }

    /// <summary>
    /// <code>
    /// string model_name                   # name of the Unity prefab model to be spawn
    /// string model_prefab_name            # this should be an .zosim json file all in text
    /// string unity_asset_bundle           # the file name of the Unity asset bundle.  If empty  uses the "default_zero_sim_assets"
    /// string unity_asset_bundle_uri       # URI to download a Unity asset bundle.  If empty  uses the "default_zero_sim_assets"
    /// string robot_namespace              # spawn robot and all ROS interfaces under this namespace
    /// geometry_msgs/Pose initial_pose     # only applied to canonical body
    /// string reference_frame              # initial_pose is defined relative to the frame of this model/body
    ///                                     # if left empty or "map", then Unity map frame is used
    ///                                     # if non-existent model/body is specified, an error is returned
    ///                                     #   and the model is not spawned
    /// ---
    /// bool success                      # return true if spawn successful
    /// string status_message             # comments if available
    /// </code>
    /// </summary>
    public class ZOSimPrefabSpawnRequest : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "zero_sim_ros/ZOSimPrefabSpawn"; } }


        /// <summary>
        /// Name of the model to be spawned.  Will be the name of the Unity GameObject.
        /// </summary>
        /// <value></value>
        public string model_name { get; set; }

        /// <summary>
        /// Name of the prefab to spawn
        /// </summary>
        /// <value></value>
        public string model_prefab_name { get; set; }

        /// <summary>
        /// The address of the asset
        /// </summary>
        /// <value></value>
        public string prefab_address { get; set; }

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


        public ZOSimPrefabSpawnRequest() {
            this.model_name = "";
            this.model_prefab_name = "";
            this.prefab_address = "";
            this.initial_pose = new PoseMessage();
            this.reference_frame = "world";
        }

        public ZOSimPrefabSpawnRequest(string model_name, string prefab_address, PoseMessage initial_pose, string reference_frame) {
            this.model_name = model_name;
            this.model_prefab_name = model_prefab_name;
            this.prefab_address = prefab_address;
            this.initial_pose = new PoseMessage();
            this.reference_frame = reference_frame;
        }
    }

    public class ZOSimPrefabSpawnResponse : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "zero_sim_ros/ZOSimPrefabSpawn"; } }

        public bool success { get; set; }
        public string status_message { get; set; }

        public ZOSimPrefabSpawnResponse() {
            this.success = true;
            this.status_message = "";
        }

        public ZOSimPrefabSpawnResponse(bool success, string status_message) {
            this.success = success;
            this.status_message = status_message;
        }

    }



    /// <summary>
    /// <code>
    /// string model_name                 # name of the Unity Model to be deleted
    /// ---
    /// bool success                      # return true if deletion is successful
    /// string status_message             # comments if available
    /// </code>
    /// </summary>
    public class ZOSimDeleteModelRequest : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "zero_sim_ros/ZOSimDeleteModel"; } }


        /// <summary>
        /// Name of the model to be deleted.  Will be the name of the Unity GameObject.
        /// </summary>
        /// <value></value>
        public string model_name { get; set; }



        public ZOSimDeleteModelRequest() {
            this.model_name = "";
        }

        public ZOSimDeleteModelRequest(string model_name) {
            this.model_name = model_name;
        }
    }

    public class ZOSimDeleteModelResponse : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "zero_sim_ros/ZOSimDeleteModel"; } }

        public bool success { get; set; }
        public string status_message { get; set; }

        public ZOSimDeleteModelResponse() {
            this.success = true;
            this.status_message = "";
        }

        public ZOSimDeleteModelResponse(bool success, string status_message) {
            this.success = success;
            this.status_message = status_message;
        }

    }

}