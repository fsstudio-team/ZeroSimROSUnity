using System;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ZO.ROS.MessageTypes.TF2;
using ZO.ROS.MessageTypes.Geometry;
using ZO.ROS.MessageTypes.Gazebo;
using ZO.ROS.MessageTypes;
using ZO.ROS.Unity.Publisher;

namespace ZO.ROS.Unity {

    /// <summary>
    /// On ROS Connect & Disconnect event type definition. 
    /// </summary>
    [System.Serializable]
    public class ZOROSBridgeConnectEvent : UnityEngine.Events.UnityEvent<ZOROSUnityManager, ZOROSBridgeConnection> { }

    /// <summary>
    /// Manage ROS with Unity specific functionality.
    /// </summary>
    [ExecuteAlways]
    public class ZOROSUnityManager : MonoBehaviour {
        public string Hostname = "localhost";
        public int Port = 9090;
        public ZOROSBridgeConnection.SerializationType _serializationType = ZOROSBridgeConnection.SerializationType.BSON;


        /// <summary>
        /// Unity event called when connected to ROS Bridge.
        /// </summary>
        /// <returns></returns>
        public ZOROSBridgeConnectEvent _connectEvent = new ZOROSBridgeConnectEvent();
        public ZOROSBridgeConnectEvent ROSBridgeConnectEvent {
            get => _connectEvent;
        }

        /// <summary>
        /// Unity event called when disconnected from ROS Bridge
        /// </summary>
        /// <returns></returns>
        public ZOROSBridgeConnectEvent _disconnectEvent = new ZOROSBridgeConnectEvent();
        public ZOROSBridgeConnectEvent ROSBridgeDisconnectEvent {
            get => _disconnectEvent;
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


        private AssetBundle _defaultZeroSimAssets = null;
        /// <summary>
        /// Default Asset Bundle readonly accessor.
        /// </summary>
        /// <value>AssetBundle</value>
        public AssetBundle DefaultAssets {
            get => _defaultZeroSimAssets;
        }

        private Queue<Tuple<SpawnModelServiceRequest, string>> _spawnModelRequests = new Queue<Tuple<SpawnModelServiceRequest, string>>();
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

                if (_defaultZeroSimAssets == null) {
                    // Load default asset bundles
                    _defaultZeroSimAssets = AssetBundle.LoadFromFile(Path.Combine(Application.streamingAssetsPath, "default_zero_sim_assets"));
                    if (_defaultZeroSimAssets == null) {
                        Debug.LogWarning("WARNING: failed to load zerosimrobots asset bundle");
                    } else {
                        Debug.Log("INFO: Load zerosimrobots asset bundle success!");
                    }
                }

                ROSBridgeConnection.Serialization = _serializationType;
                ROSBridgeConnection.OnConnectedToROSBridge = (rosBridge) => {
                    Debug.Log("INFO: Connected to ROS Bridge");

                    // advertise the transform broadcast
                    rosBridge.Advertise("/tf", _transformBroadcast.MessageType);

                    // advertise SpawnModel service
                    rosBridge.AdvertiseService<SpawnModelServiceRequest>("gazebo/spawn_urdf_model", "gazebo_msgs/SpawnModel", SpawnModelHandler);

                    // inform listeners we have connected
                    _connectEvent.Invoke(this, rosBridge);


                    return Task.CompletedTask;
                };
                ROSBridgeConnection.OnDisconnectedFromROSBridge = (rosBridge) => {
                    Debug.Log("INFO: Disconnected to ROS Bridge");

                    // inform listeners we have disconnected
                    _disconnectEvent.Invoke(this, rosBridge);

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
            ROSBridgeConnection.UnAdvertiseService("gazebo/spawn_urdf_model");
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

            // handle any spawn model requests
            while (_spawnModelRequests.Count > 0) {
                Tuple<SpawnModelServiceRequest, string> spawnRequestAndId = _spawnModelRequests.Dequeue();
                SpawnModelServiceRequest spawnRequest = spawnRequestAndId.Item1;
                string id = spawnRequestAndId.Item2;

                GameObject loadedAsset = DefaultAssets.LoadAsset<GameObject>(spawnRequest.model_name);
                if (loadedAsset != null) {
                    Vector3 position = spawnRequest.initial_pose.position.ToUnityVector3();
                    Quaternion rotation = spawnRequest.initial_pose.orientation.ToUnityQuaternion();
                    Instantiate(loadedAsset, position, rotation);

                    // report back success
                    ROSBridgeConnection.ServiceResponse<SpawnModelServiceResponse>(new SpawnModelServiceResponse() {
                        success = true,
                        status_message = "done!"
                    }, "gazebo/spawn_urdf_model", true, id);

                } else { // error loading asset
                    ROSBridgeConnection.ServiceResponse<SpawnModelServiceResponse>(new SpawnModelServiceResponse() {
                        success = false,
                        status_message = "ERROR: loading model: " + spawnRequest.model_name
                    }, "gazebo/spawn_urdf_model", false, id);

                }

            }
        }

        public void BroadcastTransform(TransformStampedMessage transformStamped) {
            if (ROSBridgeConnection.IsConnected) {
                _transformsToBroadcast.Add(transformStamped);
            }

        }

        private Task SpawnModelHandler(ZOROSBridgeConnection rosBridgeConnection, ZOROSMessageInterface msg, string id) {
            Debug.Log("INFO: ZOROSUnityManager::SpawnModelHandler");

            // queue up the spawn model because it needs to be done in the main thread Update()
            _spawnModelRequests.Enqueue(new Tuple<SpawnModelServiceRequest, string>((SpawnModelServiceRequest)msg, id));


            return Task.CompletedTask;
        }
    }

}
