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
// using ZO.ROS.MessageTypes.ZOSim;
// using ZO.ROS.MessageTypes.Gazebo;
using ZO.ROS.MessageTypes;
using ZO.ROS.Unity.Publisher;

namespace ZO.ROS.Unity {

    // [System.Serializable]
    /// <summary>
    /// On ROS Connect & Disconnect event type definition. 
    /// </summary>
    // public class ZOROSBridgeConnectEvent : UnityEngine.Events.UnityEvent<ZOROSUnityManager, ZOROSBridgeConnection> { }


    /// <summary>
    /// Manage ROS with Unity specific functionality.
    /// </summary>
    /// [ExecuteAlways]
    public class ZOROSUnityManager : MonoBehaviour {

        /// <summary>
        /// The default ROS namespace for this simulation.
        /// </summary>
        public string _namespace = "/zerosim";

        public string Namespace {
            get => _namespace;
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
        public delegate void ROSBridgeConnectionChangeHandler(object sender);
        public event ROSBridgeConnectionChangeHandler _connectEvent;
        // public ZOROSBridgeConnectEvent _connectEvent = new ZOROSBridgeConnectEvent();

        /// <summary>
        /// Unity event that is called when connected to ROS bridge.
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
        /// Unity event called when disconnected from ROS Bridge
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

        /// <summary>
        /// The singleton ROS Bridge Connection
        /// </summary>
        /// <value></value>
        public ZOROSBridgeConnection ROSBridgeConnection {
            get { return ZOROSBridgeConnection.Instance; }
        }

        private TFMessage _transformBroadcast = new TFMessage();
        private List<TransformStampedMessage> _transformsToBroadcast = new List<TransformStampedMessage>();

        private static ZOROSUnityManager _instance;

        /// <summary>
        /// Singleton access to this ROS Unity Manager.
        /// </summary>
        public static ZOROSUnityManager Instance {
            get => _instance;
        }


        // publish 
        [SerializeField] public ZOROSTransformPublisher _rootMapTransformPublisher;
        public ZOROSTransformPublisher RootMapTransform {
            get => _rootMapTransformPublisher;
            private set {
                _rootMapTransformPublisher = value;
                _rootMapTransformPublisher.FrameID = "";
                _rootMapTransformPublisher.ChildFrameID = "map";
                _rootMapTransformPublisher.UpdateRateHz = 1.0f;
                _rootMapTransformPublisher.ROSTopic = "";
                _rootMapTransformPublisher.ROSId = "";
            }
        }


        private void Awake() {
            if (_instance == null) {
                _instance = this;
                // DontDestroyOnLoad(this.gameObject);
            } else if (_instance != this) {
                Debug.LogError("ERROR: Cannot have two ZOROSUnityManager's!!!");
                Destroy(this.gameObject);
            }
        }


        // Start is called before the first frame update
        void Start() {


            if (Application.IsPlaying(gameObject) == false) { // In Editor Mode 
                if (RootMapTransform == null) { // create the root map transform
                    RootMapTransform = gameObject.AddComponent<ZOROSTransformPublisher>();
                }
            } else { // in play mode


                ROSBridgeConnection.Serialization = _serializationType;
                ROSBridgeConnection.OnConnectedToROSBridge = (rosBridge) => {
                    Debug.Log("INFO: Connected to ROS Bridge");

                    // advertise the transform broadcast
                    rosBridge.Advertise("/tf", _transformBroadcast.MessageType);

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


                    return Task.CompletedTask;
                };
                ROSBridgeConnection.OnDisconnectedFromROSBridge = (rosBridge) => {
                    Debug.Log("INFO: Disconnected to ROS Bridge");

                    // inform listeners we have disconnected
                    _disconnectEvent.Invoke(this);

                    // Unadvertise broadcast tf 
                    rosBridge.UnAdvertise("/tf");

                    return Task.CompletedTask;
                };

                // run async task.  if cannot connect wait for a couple of seconds and try again
                Task rosBridgeConnectionTask = Task.Run(async () => {
                    await ROSBridgeConnection.ConnectAsync();
                });

            }

        }

        private void OnDestroy() {
            ROSBridgeConnection.UnAdvertise("/tf");
            ROSBridgeConnection.UnAdvertiseService(_namespace + "/spawn_zosim_model");
            ROSBridgeConnection.Stop();
        }


        // Update is called once per frame
        void Update() {
            // publish map transform
            if (ROSBridgeConnection.IsConnected) {

                _transformBroadcast.transforms = _transformsToBroadcast.ToArray();
                ROSBridgeConnection.Publish<TFMessage>(_transformBroadcast, "/tf");
                _transformsToBroadcast.Clear();
            }

        }

        public void BroadcastTransform(TransformStampedMessage transformStamped) {
            if (ROSBridgeConnection.IsConnected) {
                _transformsToBroadcast.Add(transformStamped);
            }

        }

    }

}
