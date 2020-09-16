using System;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.TF2;
using ZO.ROS.MessageTypes.Geometry;
using ZO.ROS.MessageTypes.ROSGraph;
using ZO.ROS.Publisher;
using ZO.ROS.Unity.Docker;

namespace ZO.ROS.Unity {

    /// <summary>
    /// Manage ROS with Unity specific functionality.
    /// </summary>
    [ExecuteAlways]
    public class ZOROSUnityManager : MonoBehaviour {

        /// <summary>
        /// The default ROS namespace for this simulation.
        /// </summary>
        public string _namespace = "/zerosim";

        public string Namespace {
            get => _namespace;
        }

        #region ROSBridgeConnection

        /// <summary>
        /// The singleton ROS Bridge Connection
        /// </summary>
        /// <value></value>
        public ZOROSBridgeConnection ROSBridgeConnection {
            get { return ZOROSBridgeConnection.Instance; }
        }


        /// <summary>
        /// IP or hostname of the ROS Bridge
        /// </summary>
        public string Hostname = "localhost";

        /// <summary>
        /// TCP Port for the ROS bridge. Do not change unless the default ROS Bridge port has been changed.
        /// </summary>
        public int Port = 9090;

        /// <summary>
        /// JSON or BSON serialization.  Recommended to stick with BSON as it is the most efficient.
        /// </summary>
        public ZOROSBridgeConnection.SerializationType _serializationType = ZOROSBridgeConnection.SerializationType.BSON;


        /// <summary>
        /// Event handler delegate definition that is used for ROS Bridge connect & disconnect events.
        /// </summary>
        /// <returns></returns>
        public delegate void ROSBridgeConnectionChangeHandler(ZOROSUnityManager sender);
        public event ROSBridgeConnectionChangeHandler _connectEvent;
        // public ZOROSBridgeConnectEvent _connectEvent = new ZOROSBridgeConnectEvent();

        /// <summary>
        /// Event that is called when connected to ROS bridge.
        /// </summary>
        /// <value></value>
        public event ROSBridgeConnectionChangeHandler ROSBridgeConnectEvent {
            add {
                _connectEvent += value;
            }
            remove {
                _connectEvent -= value;
            }
        }

        public event ROSBridgeConnectionChangeHandler _disconnectEvent;
        /// <summary>
        /// Event called when disconnected from ROS Bridge
        /// </summary>
        /// <returns></returns>
        public event ROSBridgeConnectionChangeHandler ROSBridgeDisconnectEvent {
            add {
                _disconnectEvent += value;
            }
            remove {
                _disconnectEvent -= value;
            }
        }
        #endregion // ROSBridgeConnection

        #region ROS Docker Launch
        public bool _launchROSDocker = false;

        /// <summary>
        /// If set then launch docker image with ROS and execute ROS launch file.  
        /// </summary>
        /// <value></value>
        public bool LaunchROSDocker {
            get => _launchROSDocker;
        }
        public ZOROSLaunchParameters _rosLaunchParameters;

        /// <summary>
        /// The ROS Launch parameter scriptable object.
        /// </summary>
        /// <value></value>
        public ZOROSLaunchParameters ROSLaunchParameters {
            get => _rosLaunchParameters;
        }

        #endregion // Docker Launch


        #region Tranform Publishing

        public string _worldFrameId = "map";
        /// <summary>
        /// The name of the world frame.  Usually "map" or "world".
        /// </summary>
        /// <value></value>
        public string WorldFrameId {
            get => _worldFrameId;
        }
        private TFMessage _transformBroadcast = new TFMessage();
        private List<TransformStampedMessage> _transformsToBroadcast = new List<TransformStampedMessage>();

        // publish 
        [SerializeField] public ZOROSTransformPublisher _rootMapTransformPublisher;
        public ZOROSTransformPublisher RootMapTransform {
            get => _rootMapTransformPublisher;
            private set {
                _rootMapTransformPublisher = value;
                _rootMapTransformPublisher.FrameID = "";
                _rootMapTransformPublisher.ChildFrameID = WorldFrameId;
                _rootMapTransformPublisher.UpdateRateHz = 1.0f;
                _rootMapTransformPublisher.ROSTopic = "";
            }
        }

        /// <summary>
        /// The ROS "/tf" topic broadcast. Provides an easy way to publish coordinate frame transform information. 
        /// </summary>
        /// <param name="transformStamped"></param> 
        public void BroadcastTransform(TransformStampedMessage transformStamped) {
            if (ROSBridgeConnection.IsConnected) {
                _transformsToBroadcast.Add(transformStamped);
            }

        }

        #endregion // Transform Publishing


        #region Singleton
        private static ZOROSUnityManager _instance;

        /// <summary>
        /// Singleton access to this ROS Unity Manager.
        /// </summary>
        public static ZOROSUnityManager Instance {
            get => _instance;
        }
            
        #endregion // Singleton


        #region AssetBundle Management

        private AssetBundle _defaultZeroSimAssets = null;
        /// <summary>
        /// Default Asset Bundle readonly accessor.
        /// </summary>
        /// <value>AssetBundle</value>
        public AssetBundle DefaultAssets {
            get {
                if (_defaultZeroSimAssets == null) {
                    // Load default asset bundles
                    _defaultZeroSimAssets = AssetBundle.LoadFromFile(Path.Combine(Application.streamingAssetsPath, "default_zero_sim_assets"));
                    if (_defaultZeroSimAssets == null) {
                        Debug.LogWarning("WARNING: failed to load default_zero_sim_assets asset bundle");
                    } else {
                        Debug.Log("INFO: Load default_zero_sim_assets asset bundle success!");
                    }

                }
                return _defaultZeroSimAssets;
            }
        }
        #endregion

        #region Simulation Clock

        ClockMessage _clockMessage = new ClockMessage();
        
        /// <summary>
        /// sim
        /// </summary>
        /// <value></value>
        public ClockMessage Clock {
            get => _clockMessage;
        }
        #endregion // Simulation Clock


        private void Awake() {
            if (_instance == null) {
                _instance = this;
                // DontDestroyOnLoad(this.gameObject);
            } else if (_instance != this) {
                Debug.LogError("ERROR: Cannot have two ZOROSUnityManager's!!!");
                Destroy(this.gameObject);
            }

            if (_defaultZeroSimAssets == null) {
                // Load default asset bundles
                _defaultZeroSimAssets = AssetBundle.LoadFromFile(Path.Combine(Application.streamingAssetsPath, "default_zero_sim_assets"));
                if (_defaultZeroSimAssets == null) {
                    Debug.LogWarning("WARNING: failed to load zerosimrobots asset bundle");
                } else {
                    Debug.Log("INFO: Load zerosimrobots asset bundle success!");
                }
            }

        }

        // Start is called before the first frame update
        void Start() {


            if (Application.IsPlaying(gameObject) == false) { // In Editor Mode 
                if (RootMapTransform == null) { // create the root map transform if doesn't exist
                    RootMapTransform = gameObject.AddComponent<ZOROSTransformPublisher>();
                }

                // 
                if (_launchROSDocker == true) {
                    string launchCommand = $"";
                    // ZO.Editor.ZODockerManager.DockerRun("zosim", launchCommand);
                }
            } else { // in play mode


                ROSBridgeConnection.Serialization = _serializationType;
                ROSBridgeConnection.ROSBridgeConnectEvent += delegate(ZOROSBridgeConnection rosBridge) {
                    Debug.Log("INFO: Connected to ROS Bridge");

                    // advertise the transform broadcast
                    rosBridge.Advertise("/tf", _transformBroadcast.MessageType);

                    // advertise the simulation clock
                    rosBridge.Advertise("/clock", Clock.MessageType);

                    // advertise SpawnModel service
                    // rosBridge.AdvertiseService<SpawnModelServiceRequest>("gazebo/spawn_urdf_model", "gazebo_msgs/SpawnModel", (bridge, msg, id) => {
                    //     Debug.Log("INFO: got gazebo/spawn_urdf_model");
                    //     // report back success
                    //     ROSBridgeConnection.ServiceResponse<ZOSimSpawnServiceResponse>(new ZOSimSpawnServiceResponse() {
                    //         success = true,
                    //         status_message = "done!"
                    //     }, "gazebo/spawn_urdf_model", true, id);

                    //     return Task.CompletedTask;
                    // });
                    // rosBridge.AdvertiseService<ZOSimSpawnServiceRequest>(_namespace + "/spawn_zosim_model", "zero_sim_ros/ZOSimSpawn", SpawnModelHandler);

                    try {
                        // inform listeners we have connected
                        _connectEvent.Invoke(this);
                    } catch (System.Exception e) {
                        Debug.LogError("ERROR: ZOROSUnityManager Connected Invoke: " + e.ToString());
                    }
                };

                ROSBridgeConnection.ROSBridgeDisconnectEvent += delegate (ZOROSBridgeConnection rosBridge) {
                    Debug.Log("INFO: Disconnected to ROS Bridge");

                    // inform listeners we have disconnected
                    _disconnectEvent?.Invoke(this);

                    // Unadvertise broadcast tf 
                    rosBridge.UnAdvertise("/tf");

                    // Unadvertise simulation clock
                    rosBridge.UnAdvertise("/clock");
                };

                // run async task.  if cannot connect wait for a couple of seconds and try again
                Task rosBridgeConnectionTask = Task.Run(async () => {
                    await ROSBridgeConnection.ConnectAsync();
                });

            }

        }

        private void OnDestroy() {
            ROSBridgeConnection.UnAdvertise("/tf");
            // ROSBridgeConnection.UnAdvertiseService(_namespace + "/spawn_zosim_model");
            ROSBridgeConnection.Stop();
            if (_defaultZeroSimAssets != null) {
                _defaultZeroSimAssets.Unload(true);
                _defaultZeroSimAssets = null;
            }
        }


        // Update is called once per frame
        void Update() {
            // publish map transform
            if (ROSBridgeConnection.IsConnected) {

                // transform broadcast
                _transformBroadcast.transforms = _transformsToBroadcast.ToArray();
                ROSBridgeConnection.Publish<TFMessage>(_transformBroadcast, "/tf");
                _transformsToBroadcast.Clear();

                // simulation clock
                Clock.Update();
                ROSBridgeConnection.Publish<ClockMessage>(Clock, "/clock");
            }

        }


    }

}
