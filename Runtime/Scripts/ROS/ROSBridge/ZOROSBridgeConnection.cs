using System.Net;
using System.Net.NetworkInformation;
using System;
using System.IO;
using System.Text;
using System.Net.Sockets;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Newtonsoft.Json.Bson;
using ZO.ROS.MessageTypes;
using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.ROSAPI;

namespace ZO.ROS {

    /// <summary>
    /// ROS Bridge Connection manager singleton.  
    /// Supports TCP connection. TODO: UDP & WebSocket
    /// Supports BSON & JSON encoding.    
    /// 
    /// ROS operations:
    ///
    ///     advertise – advertise that you are publishing a topic
    ///     unadvertise – stop advertising that you are publishing topic publish - a published ROS-message
    ///     subscribe - a request to subscribe to a topic
    ///     unsubscribe - a request to unsubscribe from a topic
    ///     call_service - a service call
    ///     advertise_service - advertise an external service server
    ///     unadvertise_service - unadvertise an external service server
    ///     service_request - a service request
    ///     service_response - a service response
    /// </summary>
    /// <see>
    /// See: https://github.com/biobotus/rosbridge_suite
    /// See: https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md
    /// </see>
    /// <example><code>
    /// # run a ROS Bridge Connection TCP server for BSON encoding:
    /// roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=true
    /// </code></example>
    /// <example><code>
    /// ZOROSBridgeConnection.Instance.Serialization = ZOROSBridgeConnection.SerializationType.BSON;
    /// ZOROSBridgeConnection.Instance.OnConnectedToROSBridge = (controller) => {
    ///     Debug.Log("INFO: Connected to ROS Bridge");

    ///     // test advertising
    ///     controller.Advertise("cmd_vel_zero", "geometry_msgs/Twist");

    ///     // test subscription
    ///     controller.Subscribe<Int32Message>("subscriber_1", "client_count", "std_msgs/Int32", (connection, msg) => {
    ///         Debug.Log("INFO:  Received subscription: " + ((Int32Message)msg).data);
    ///         return Task.CompletedTask;
    ///     });

    ///     // test subscription
    ///     // See: `zero_sim_ros/scripts/zo_publisher_test.py`
    ///     // `rosrun zero_sim_ros zo_publisher_test.py`
    ///     controller.Subscribe<StringMessage>("subscriber_1", "chatter", "std_msgs/String", (cont, msg) => {
    ///         Debug.Log("INFO:  Received subscription: " + ((StringMessage)msg).data);
    ///         return Task.CompletedTask;
    ///     });

    ///     return Task.CompletedTask;
    /// };
    /// ZOROSBridgeConnection.Instance.OnDisconnectedFromROSBridge = (controller) => {
    ///     Debug.Log("INFO: Disconnected to ROS Bridge");
    ///     controller.UnAdvertise("cmd_vel_zero");
    ///     return Task.CompletedTask;
    /// };

    /// // connect to ROS Bridge asynchronously. 
    /// Task controllerSubscriptionTask = Task.Run(async () => {
    ///     await ZOROSBridgeConnection.Instance.ConnectAsync();
    /// });
    /// </code></example>
    public class ZOROSBridgeConnection {

        #region Singleton

        // Singleton: See https://csharpindepth.com/articles/singleton
        private static readonly ZOROSBridgeConnection _instance = new ZOROSBridgeConnection();
        static ZOROSBridgeConnection() { }
        private ZOROSBridgeConnection() { }
        public static ZOROSBridgeConnection Instance { get => _instance; }

        #endregion // Singleton

        public enum SerializationType {
            JSON,
            BSON
        };



        public SerializationType Serialization { get; set; }
        private TcpClient _tcpClient;

        public delegate void ROSBridgeConnectionChangeHandler(ZOROSBridgeConnection sender);
        private event ROSBridgeConnectionChangeHandler _connectEvent;
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


        /// <summary>
        /// Hostname or IP addresss
        /// </summary>
        /// <value></value>
        public string Hostname { get; set; } = "localhost";

        /// <summary>
        /// TCP Port. Do not recommend changing.
        /// </summary>
        /// <value></value>
        public int Port { get; set; } = 9090;
        private bool _isConnected = false;

        /// <summary>
        /// Checks if the ROS Bridge is connected.
        /// </summary>
        /// <value></value>
        public bool IsConnected {
            get => _isConnected;
        }


        /// <summary>
        /// Generic wrapper class message publishing over ros bridge.
        /// </summary>
        /// <see>
        /// See: https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md
        /// <see>        
        internal class GenericMessagePublish<T> where T : ZOROSMessageInterface {

            public string op { get; set; }
            public string id { get; set; }

            public string topic { get; set; }   // required
            public T msg { get; set; }          // required

            internal GenericMessagePublish(string id, string topic, T msg) {
                this.id = id;
                this.op = "publish";
                this.topic = topic;
                this.msg = msg;
            }

            internal GenericMessagePublish() {

            }
        }

        /// <summary>
        /// Generic wrapper class for ROS bridge service response messages.
        /// </summary>
        /// <typeparam name="T">ZOROSMessageInterface</typeparam>
        internal class GenericServiceResponse<T> where T : ZOROSMessageInterface {
            public string op { get; set; }
            public string service { get; set; }
            public T values { get; set; }
            public bool result { get; set; }
            public string id { get; set; }
            internal GenericServiceResponse(string service, bool result, T values, string id) {
                this.op = "service_response";
                this.service = service;
                this.values = values;
                this.result = result;
                this.id = id;
            }

            internal GenericServiceResponse() {
                this.op = "service_response";
                this.service = "";
                this.result = false;
                this.id = "";
            }

        }


        internal abstract class MessageHandler {
            [Newtonsoft.Json.JsonIgnore]
            public Func<ZOROSBridgeConnection, ZOROSMessageInterface, Task> OnMessageReceivedHandler;


            public abstract ZOROSMessageInterface DeserializeBSON(byte[] data);
            public abstract ZOROSMessageInterface DeserializeJSON(string json);

        }

        /// <summary>
        /// NOTE: difference between MessageHandler and this is the extra "string" that returns the "id" in OnServiceCallHandler
        /// </summary>
        internal abstract class ServiceCallHandler {
            [Newtonsoft.Json.JsonIgnore]
            public Func<ZOROSBridgeConnection, ZOROSMessageInterface, string, Task> OnServiceCallHandler;
            public abstract ZOROSMessageInterface DeserializeBSON(byte[] data);
            public abstract ZOROSMessageInterface DeserializeJSON(string json);

        }


        /// <summary>
        /// NOTE: We implement a class instead of generating JSON because BSON encoding really requires a class because it cannot
        /// infer types sizes like <c>int</c>, so it will always be an <c>Int64</c> from just JSON, but with a object it can work
        /// out the type.
        /// 
        /// This command subscribes the client to the specified topic. It is recommended that if the client has multiple components subscribing to the same topic, that each component makes its own subscription request providing an ID. That way, each can individually unsubscribe and rosbridge can select the correct rate at which to send messages.

        ///     type – the (expected) type of the topic to subscribe to. If left off, type will be inferred, and if the topic doesn't exist then the command to subscribe will fail
        ///     topic – the name of the topic to subscribe to
        ///     throttle_rate – the minimum amount of time (in ms) that must elapse between messages being sent. Defaults to 0
        ///     queue_length – the size of the queue to buffer messages. Messages are buffered as a result of the throttle_rate. Defaults to 1.
        ///     id – if specified, then this specific subscription can be unsubscribed by referencing the ID.
        ///     fragment_size – the maximum size that a message can take before it is to be fragmented.
        ///     compression – an optional string to specify the compression scheme to be used on messages. Valid values are "none" and "png"
        ///
        /// If queue_length is specified, then messages are placed into the queue before being sent. Messages are sent from the head of the queue. If the queue gets full, the oldest message is removed and replaced by the newest message.
        ///
        /// If a client has multiple subscriptions to the same topic, then messages are sent at the lowest throttle_rate, with the lowest fragmentation size, and highest queue_length. It is recommended that the client provides IDs for its subscriptions, to enable rosbridge to effectively choose the appropriate fragmentation size and publishing rate.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        internal class GenericMessageSubscribe<T> : MessageHandler where T : ZOROSMessageInterface {

            public string op { get; set; }
            public string id { get; set; }

            public string topic { get; set; }       // required
            public string type { get; set; }        // optional
            public int throttle_rate { get; set; }  // optional
            public int queue_length { get; set; }   // optional
            public int fragment_size { get; set; }  // optional
            public string compression { get; set; } // optional

            internal GenericMessageSubscribe(string id, string topic, string type, Func<ZOROSBridgeConnection, ZOROSMessageInterface, Task> OnReceiveDelegate, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none") {
                this.id = id;
                this.op = "subscribe";
                this.topic = topic;
                this.type = type;
                this.throttle_rate = throttle_rate;
                this.queue_length = queue_length;
                this.fragment_size = fragment_size;
                this.compression = compression;
                this.OnMessageReceivedHandler = OnReceiveDelegate;
            }


            public override ZOROSMessageInterface DeserializeBSON(byte[] data) {
                GenericMessagePublish<T> receivedPublishMessage = new GenericMessagePublish<T>();
                MemoryStream memoryStream = new MemoryStream(data);
                using (BsonDataReader reader = new BsonDataReader(memoryStream)) {
                    JsonSerializer serializer = new JsonSerializer();
                    receivedPublishMessage = serializer.Deserialize<GenericMessagePublish<T>>(reader);
                }

                return receivedPublishMessage.msg;
            }

            public override ZOROSMessageInterface DeserializeJSON(string json) {
                ZOROSMessageInterface messageObject = JsonConvert.DeserializeObject<T>(json);
                return messageObject;
            }

        }

        internal class GenericServiceAdvertise<T> : ServiceCallHandler where T : ZOROSMessageInterface {

            public string op { get; set; }
            public string service { get; set; }

            public string type { get; set; }        // optional
            internal GenericServiceAdvertise(string service, string type, Func<ZOROSBridgeConnection, ZOROSMessageInterface, string, Task> onServiceCallHandler) {
                this.service = service;
                this.op = "advertise_service";
                this.type = type;
                this.OnServiceCallHandler = onServiceCallHandler;
            }


            public override ZOROSMessageInterface DeserializeBSON(byte[] data) {
                //BUGBUG: is this BridgeMessagePublish correct?  Shouldn't it be BridgeServiceResponse?
                GenericMessagePublish<T> receivedPublishMessage = new GenericMessagePublish<T>();
                MemoryStream memoryStream = new MemoryStream(data);
                using (BsonDataReader reader = new BsonDataReader(memoryStream)) {
                    JsonSerializer serializer = new JsonSerializer();
                    receivedPublishMessage = serializer.Deserialize<GenericMessagePublish<T>>(reader);
                }

                return receivedPublishMessage.msg;
            }

            public override ZOROSMessageInterface DeserializeJSON(string json) {
                ZOROSMessageInterface messageObject = JsonConvert.DeserializeObject<T>(json);
                return messageObject;
            }

        }

        /// <summary>
        /// Generic wrapper class for ROS bridge service calls.
        /// <code>
        ///{ "op": "call_service",
        /// (optional) "id": <string>,
        ///  "service": <string>,
        ///  (optional) "args": <list<json>>,
        ///  (optional) "fragment_size": <int>,
        ///  (optional) "compression": <string>
        ///}
        /// </code>
        /// service – the name of the service to call
        /// args – if the service has no args, then args does not have to be provided, though an empty list is equally acceptable. Args should be a list of json objects representing the arguments to the service
        /// id – an optional id to distinguish this service call
        /// fragment_size – the maximum size that the response message can take before it is fragmented
        /// compression – an optional string to specify the compression scheme to be used on messages. Valid values are "none" and "png"
        /// </summary>
        /// <typeparam name="T">The calling message</typeparam>
        /// <typeparam name="R">The response message</typeparam>
        internal class GenericCallService<T, R> : MessageHandler where T : ZOROSMessageInterface where R : ZOROSMessageInterface {
            public string op { get; set; }
            public string id { get; set; }
            public string service { get; set; }
            public T args { get; set; }
            internal GenericCallService(string service, T args, string id, Func<ZOROSBridgeConnection, ZOROSMessageInterface, Task> OnReceiveDelegate) {
                this.op = "call_service";
                this.service = service;
                this.args = args;
                this.id = id;
                this.OnMessageReceivedHandler = OnReceiveDelegate;
            }

            internal GenericCallService() {
                this.op = "call_service";
                this.service = service;
                this.args = args;
                this.id = id;
            }

            public override ZOROSMessageInterface DeserializeBSON(byte[] data) {
                GenericServiceResponse<R> receivedPublishMessage = new GenericServiceResponse<R>();
                MemoryStream memoryStream = new MemoryStream(data);
                using (BsonDataReader reader = new BsonDataReader(memoryStream)) {
                    JsonSerializer serializer = new JsonSerializer();
                    receivedPublishMessage = serializer.Deserialize<GenericServiceResponse<R>>(reader);
                }

                return receivedPublishMessage.values;
            }

            public override ZOROSMessageInterface DeserializeJSON(string json) {
                ZOROSMessageInterface messageObject = JsonConvert.DeserializeObject<R>(json);
                return messageObject;
            }


        }


        /// <summary>
        /// Dictionary string:topic list:callbacks
        /// callback: bridge connection, JSON msg
        /// </summary>
        /// <returns></returns>
        private Dictionary<string, List<MessageHandler>> _subscribers = new Dictionary<string, List<MessageHandler>>();

        /// <summary>
        /// ROS service provider callbacks
        /// </summary>
        /// <returns></returns>
        private Dictionary<string, ServiceCallHandler> _serviceProviders = new Dictionary<string, ServiceCallHandler>();

        // private Dictionary<string, Dictionary<string, MessageHandler>> _serviceCallResponseHandlers = new Dictionary<string, Dictionary<string, MessageHandler>>();

        private Queue<MessageHandler> _serviceCallResponseHandlers = new Queue<MessageHandler>();
        /// <summary>
        /// Asynchronously connect to ROS Bridge server.
        /// </summary>
        /// <returns>Task</returns>
        /// <example><code>
        /// // run async task.  if cannot connect wait for a couple of seconds and try again
        /// Task rosBridgeConnectionTask = Task.Run(async () => {
        ///    await _rosBridgeConnection.ConnectAsync();
        /// });
        /// </code></example>
        public async Task ConnectAsync() {

            _isConnected = false;
            _tcpClient = new TcpClient();
            while (_isConnected == false) {
                try {
                    IPAddress[] iPAddresses = Dns.GetHostAddresses(Hostname);
                    await _tcpClient.ConnectAsync(iPAddresses, Port);
                    await Task.Delay(600); // pause a short bit for things to settle
                    _isConnected = true;
                } catch (SocketException e) {
                    Debug.Log("WARNING: ZOROSBridgeConnection SocketException: " + e.ToString());
                    await Task.Delay(5000); // wait for 5 seconds
                }
            }

            Debug.Log("INFO: ZOROSBridgeConnection::ConnectAsync connected...");

            // inform listeners of connection
            try {
               _connectEvent.Invoke(this);
            } catch (Exception e) {
                Debug.LogError("ERROR: ConnectAsync::OnConnectedToROSBridge: " + e.ToString());

            }

            await ClientReadAsync();

        }

        /// <summary>
        /// Stops ROS Bridge TCP Connection.  
        /// </summary>
        /// <returns></returns>
        public async void Stop() {
            Debug.Log("INFO: ZOROSBridgeConnection::Stop Requested");

            // inform everyone we are disconnecting
            _disconnectEvent?.Invoke(this);

            if (_isConnected == true && _tcpClient != null) {
                _tcpClient.GetStream().Close();
                _tcpClient.Close();
                _tcpClient = null;
                _isConnected = false;
            }
        }


        /// <summary>
        /// Advertise a ROS message topic.
        /// </summary>
        /// <param name="topic">ROS topic.  For example: "/cmd_vel"</param>
        /// <param name="type">ROS message type. For example "geometry_msgs/Twist"</param>
        /// <param name="id">ROS node id.  Default "zero_sim_unity"</param>
        /// <returns></returns>        
        public async void Advertise(string topic, string type, string id = "zero_sim_unity") {
            JObject advertiseJSON = new JObject(
                new JProperty("op", "advertise"),
                new JProperty("topic", topic),
                new JProperty("type", type),
                new JProperty("id", id)
            );
            if (Serialization == SerializationType.JSON) {
                await SendJSONStringAsync(advertiseJSON.ToString(Formatting.None));
            } else if (Serialization == SerializationType.BSON) {
                await SendBSONAsync(advertiseJSON);
            }
        }

        /// <summary>
        /// Unadvertise a ROS message topic.
        /// </summary>
        /// <param name="topic"></param>
        /// <returns></returns>
        public async void UnAdvertise(string topic) {
            JObject unAdvertiseJSON = new JObject(
                new JProperty("op", "unadvertise"),
                new JProperty("topic", topic)
            );
            if (Serialization == SerializationType.JSON) {
                await SendJSONStringAsync(unAdvertiseJSON.ToString(Formatting.None));
            } else if (Serialization == SerializationType.BSON) {
                await SendBSONAsync(unAdvertiseJSON);
            }

        }

        /// <summary>
        /// This command subscribes the client to the specified topic. It is recommended that if the client has
        /// multiple components subscribing to the same topic, that each component makes its own subscription 
        /// request providing an ID. That way, each can individually unsubscribe and rosbridge can select the 
        /// correct rate at which to send messages.
        ///
        ///     type – the (expected) type of the topic to subscribe to. If left off, type will be inferred, and if
        ///            the topic doesn't exist then the command to subscribe will fail
        ///     topic – the name of the topic to subscribe to
        ///     throttle_rate – the minimum amount of time (in ms) that must elapse between messages being sent. 
        ///                     Defaults to 0
        ///     queue_length – the size of the queue to buffer messages. Messages are buffered as a result of the 
        ///                      throttle_rate. Defaults to 1.
        ///     id – if specified, then this specific subscription can be unsubscribed by referencing the ID.
        ///     fragment_size – the maximum size that a message can take before it is to be fragmented.
        ///     compression – an optional string to specify the compression scheme to be used on messages. Valid
        ///                   values are "none" and "png"
        ///
        /// If queue_length is specified, then messages are placed into the queue before being sent. Messages are 
        /// sent from the head of the queue. If the queue gets full, the oldest message is removed and replaced by 
        /// the newest message.
        ///
        /// If a client has multiple subscriptions to the same topic, then messages are sent at the lowest 
        /// throttle_rate, with the lowest fragmentation size, and highest queue_length. It is recommended that the 
        /// client provides IDs for its subscriptions, to enable rosbridge to effectively choose the appropriate 
        /// fragmentation size and publishing rate.
        /// </summary>
        /// <param name="id">f specified, then this specific subscription can be unsubscribed by referencing the ID.</param>
        /// <param name="topic">the name of the topic to subscribe to</param>
        /// <param name="type">ROS message type</param>
        /// <param name="subscriptionCallback">Called when message is received.</param>
        /// <typeparam name="T">ZOROSMessageInterface</typeparam>
        /// <returns></returns>
        public async void Subscribe<T>(string id, string topic, string type, Func<ZOROSBridgeConnection, ZOROSMessageInterface, Task> subscriptionCallback) where T : ZOROSMessageInterface {

            GenericMessageSubscribe<T> rosBridgeSubscription = new GenericMessageSubscribe<T>(id, topic, type, subscriptionCallback);

            // build up the subscriber callback dictionary
            if (_subscribers.ContainsKey(topic) == false) {
                // if no subscribers create a list of subscribers
                _subscribers.Add(topic, new List<MessageHandler>());
            }
            // add the subscriber to the list
            _subscribers[topic].Add(rosBridgeSubscription);
            if (Serialization == SerializationType.JSON) {
                await SendJSONStringAsync(JsonConvert.SerializeObject(rosBridgeSubscription));
            } else if (Serialization == SerializationType.BSON) {

                MemoryStream memoryStream = new MemoryStream();
                BsonDataWriter writer = new BsonDataWriter(memoryStream);
                JsonSerializer serializer = new JsonSerializer();
                serializer.Serialize(writer, rosBridgeSubscription);

                await SendBSONAsync(memoryStream);
            }
        }

        /// <summary>
        /// Unsubscribe a ROS message topic.
        /// </summary>
        /// <param name="topic"></param>
        /// <returns></returns>
        public async void Unsubscribe(string id, string topic) {
            JObject unsubscribeJSON = new JObject(
                new JProperty("op", "unsubscribe"),
                new JProperty("id", id),
                new JProperty("topic", topic)
            );
            if (Serialization == SerializationType.JSON) {
                await SendJSONStringAsync(unsubscribeJSON.ToString(Formatting.None));
            } else if (Serialization == SerializationType.BSON) {
                await SendBSONAsync(unsubscribeJSON);
            }

        }




        /// <summary>
        /// Publish message to topic.
        /// </summary>
        /// <param name="message">ROS Message type</param>
        /// <param name="topic">ROS Topic</param>
        /// <param name="id">Unique ID of sender.</param>
        /// <typeparam name="T">ROS Message derived from ZOROSMessageInterface</typeparam>
        /// <returns></returns>
        public async void Publish<T>(T message, string topic, string id = "zero_sim_unity") where T : ZOROSMessageInterface {

            if (IsConnected == false) {
                Debug.LogWarning("WARNING: attempting to publish though we are not connected to ROS bridge");
            }
            if (Serialization == SerializationType.JSON) {
                JObject publishJSON = new JObject(
                    new JProperty("op", "publish"),
                    new JProperty("topic", topic),
                    new JProperty("msg", JObject.FromObject(message))
                );

                await SendJSONStringAsync(publishJSON.ToString(Formatting.None));
            } else if (Serialization == SerializationType.BSON) {

                GenericMessagePublish<T> publishCommunication = new GenericMessagePublish<T>(id, topic, message);


                MemoryStream memoryStream = new MemoryStream();
                BsonDataWriter writer = new BsonDataWriter(memoryStream);
                JsonSerializer serializer = new JsonSerializer();
                serializer.Serialize(writer, publishCommunication);

                await SendBSONAsync(memoryStream);
            }
        }

        /// <summary>
        /// Advertises an external ROS service server. Requests come to the client via Call Service.
        /// </summary>
        /// <param name="service">The name of the service to advertise.</param>
        /// <param name="type">The advertised service message type.</param>
        /// <returns></returns>
        public async void AdvertiseService<T>(string service, string type, Func<ZOROSBridgeConnection, ZOROSMessageInterface, string, Task> serviceRequest) where T : ZOROSMessageInterface {
            JObject advertiseJSON = new JObject(
                new JProperty("op", "advertise_service"),
                new JProperty("service", service),
                new JProperty("type", type)
            );

            GenericServiceAdvertise<T> rosBridgeService = new GenericServiceAdvertise<T>(service, type, serviceRequest);

            _serviceProviders.Add(service, rosBridgeService);

            if (Serialization == SerializationType.JSON) {
                await SendJSONStringAsync(JsonConvert.SerializeObject(rosBridgeService));
            } else if (Serialization == SerializationType.BSON) {

                MemoryStream memoryStream = new MemoryStream();
                BsonDataWriter writer = new BsonDataWriter(memoryStream);
                JsonSerializer serializer = new JsonSerializer();
                serializer.Serialize(writer, rosBridgeService);

                await SendBSONAsync(memoryStream);
            }

        }

        /// <summary>
        /// Stops advertising an external ROS service server
        /// </summary>
        /// <param name="service">the name of the service to unadvertise</param>
        /// <returns></returns>
        public async void UnAdvertiseService(string service) {
            JObject advertiseJSON = new JObject(
                new JProperty("op", "unadvertise_service"),
                new JProperty("service", service)
            );
            if (Serialization == SerializationType.JSON) {
                await SendJSONStringAsync(advertiseJSON.ToString(Formatting.None));
            } else if (Serialization == SerializationType.BSON) {
                await SendBSONAsync(advertiseJSON);
            }
        }


        /// <summary>
        /// Send a response to a service call. NOTE: id parameter is very important. 
        /// </summary>
        /// <param name="response_message"> the return values. If the service had no return values, then this field can be omitted (and will be by the rosbridge server)</param>
        /// <param name="service">the name of the service that was called</param>
        /// <param name="result"></param>
        /// <param name="id">if an ID was provided to the service request, then the service response will contain the ID</param>
        /// <typeparam name="T">Response message</typeparam>
        /// <returns></returns>
        public async void ServiceResponse<T>(T response_message, string service, bool result, string id) where T : ZOROSMessageInterface {


            if (Serialization == SerializationType.JSON) {
                JObject serviceResponseJSON = new JObject(
                    new JProperty("op", "service_response"),
                    new JProperty("service", service),
                    new JProperty("values", JObject.FromObject(response_message)),
                    new JProperty("result", result)
                );
                if (id.Length > 0) {
                    serviceResponseJSON.Add("id", id);
                }

                await SendJSONStringAsync(serviceResponseJSON.ToString(Formatting.None));
            } else if (Serialization == SerializationType.BSON) {

                GenericServiceResponse<T> serviceResponse = new GenericServiceResponse<T>(service, result, response_message, id);

                MemoryStream memoryStream = new MemoryStream();
                BsonDataWriter writer = new BsonDataWriter(memoryStream);
                JsonSerializer serializer = new JsonSerializer();
                serializer.Serialize(writer, serviceResponse);

                await SendBSONAsync(memoryStream);
            }

        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="calling_message">The calling message</param>
        /// <param name="service">Service name</param>
        /// <param name="id">Unique ID. Important to be Unique!</param>
        /// <param name="serviceCallResponse">Callback when service responds</param>
        /// <typeparam name="T">Service call message type</typeparam>
        /// <typeparam name="R">Service response message type</typeparam>
        /// <returns></returns>
        public async void CallService<T, R>(T calling_message, string service, string id, Func<ZOROSBridgeConnection, ZOROSMessageInterface, Task> serviceCallResponse) where T : ZOROSMessageInterface where R : ZOROSMessageInterface {
            if (IsConnected == false) {
                Debug.LogWarning("WARNING: attempting to call service though we are not connected to ROS bridge");
            }
            GenericCallService<T, R> callService = new GenericCallService<T, R>(service, calling_message, id, serviceCallResponse);

            // if (_serviceCallResponseHandlers.ContainsKey(service) == false) {
            //     _serviceCallResponseHandlers.Add(service, new Dictionary<string, MessageHandler>());
            // }
            // _serviceCallResponseHandlers[service].Add(id, callService);
            _serviceCallResponseHandlers.Enqueue(callService);

            // if the first service call
            if (_serviceCallResponseHandlers.Count == 1) {
                if (Serialization == SerializationType.JSON) {
                    await SendJSONStringAsync(JsonConvert.SerializeObject(callService));
                } else if (Serialization == SerializationType.BSON) {

                    MemoryStream memoryStream = new MemoryStream();
                    BsonDataWriter writer = new BsonDataWriter(memoryStream);
                    JsonSerializer serializer = new JsonSerializer();
                    serializer.Serialize(writer, callService);

                    await SendBSONAsync(memoryStream);
                }

            }

        }



        private async Task ClientReadAsync() {
            Debug.Log("INFO: ZOROSBridgeConnection::ClientReadAsync Start");
            byte[] buffer = new byte[1024 * 100];  // BUGBUG: hardwired buffer.  TODO: handle when we get messages larger then the buffer!

            while (_isConnected) {
                int bytesRead = -1;
                try {

                    bytesRead = await _tcpClient.GetStream().ReadAsync(buffer, 0, buffer.Length);
                    // Debug.Log("INFO: ReadAsync read byte count" + bytesRead.ToString());
                    if (bytesRead > 0) {

                        if (Serialization == SerializationType.JSON) {
                            // check if we have valid JSON by checking if we have the '{' & '}'
                            if (buffer[0] == '{' && buffer[bytesRead - 1] == '}') {
                                string msg = System.Text.Encoding.UTF8.GetString(buffer, 0, bytesRead);
                                JObject msgJSON = JObject.Parse(msg);
                                string topic = msgJSON["topic"].Value<string>();
                                foreach (var subscriber in _subscribers[topic]) {
                                    ZOROSMessageInterface message = subscriber.DeserializeJSON(msgJSON["msg"].ToString());
                                    await subscriber.OnMessageReceivedHandler(this, message);
                                    // await subscriberCallback(this, msgJSON["msg"].ToString());
                                }
                                // Debug.Log("INFO: ReadAsync string: " + msg);
                            } else {
                                Debug.LogWarning("WARNING: ZOROSBridgeConnection::ClientReadAsync invalid JSON");
                            }

                        } else if (Serialization == SerializationType.BSON) {

                            // TODO: we have to deserialize to JSON because we do not know the type of the object
                            // coming in.  Would be faster to be able to "peek" into the BSON and then deserialize...
                            MemoryStream memStream = new MemoryStream(buffer);
                            JObject msgJSON;
                            using (BsonDataReader reader = new BsonDataReader(memStream)) {
                                msgJSON = (JObject)JToken.ReadFrom(reader);
                            }

                            // switch on ROS Bridge operation
                            string op = msgJSON["op"].Value<string>();
                            if (op == "publish") {
                                string topic = msgJSON["topic"].Value<string>();
                                foreach (var subscriber in _subscribers[topic]) {
                                    // deserialize bson message and pass on to any subscribers
                                    ZOROSMessageInterface message = subscriber.DeserializeJSON(msgJSON["msg"].ToString());
                                    await subscriber.OnMessageReceivedHandler(this, message);
                                }
                            } else if (op == "call_service") {
                                string service = msgJSON["service"].Value<string>();
                                var serviceHandler = _serviceProviders[service];
                                ZOROSMessageInterface message = serviceHandler.DeserializeJSON(msgJSON["args"].ToString());
                                string id = msgJSON["id"].Value<string>();
                                await serviceHandler.OnServiceCallHandler(this, message, id);
                            } else if (op == "service_response") {
                                // string jsonString = msgJSON.ToString(Formatting.None);

                                string service = msgJSON["service"].Value<string>();
                                string id = msgJSON["id"].Value<string>();
                                Debug.Log("INFO: ReadAsync::service_response: service: " + service + " id: " + id + " values: " + msgJSON["values"].ToString());
                                // var serviceResponseHandler = _serviceCallResponseHandlers[service][id];
                                MessageHandler serviceResponseHandler = _serviceCallResponseHandlers.Dequeue();

                                // get the message.  the message defaults to "empty" of no "values" key.
                                ZOROSMessageInterface message = new EmptyServiceRespone();
                                if (msgJSON.ContainsKey("values") == true) {
                                    message = serviceResponseHandler.DeserializeJSON(msgJSON["values"].ToString());
                                }

                                await serviceResponseHandler.OnMessageReceivedHandler(this, message);

                                // dispatch anymore service calls in the queue
                                MessageHandler nextServiceCall = _serviceCallResponseHandlers.Peek();
                                if (nextServiceCall != null) {
                                    if (Serialization == SerializationType.JSON) {
                                        await SendJSONStringAsync(JsonConvert.SerializeObject(nextServiceCall));
                                    } else if (Serialization == SerializationType.BSON) {

                                        MemoryStream memoryStream = new MemoryStream();
                                        BsonDataWriter writer = new BsonDataWriter(memoryStream);
                                        JsonSerializer serializer = new JsonSerializer();
                                        serializer.Serialize(writer, nextServiceCall);

                                        await SendBSONAsync(memoryStream);
                                    }
                                }

                            } else {
                                Debug.LogWarning("WARNING: Unhandle ROS Bridge operation: " + op);
                            }

                            // Debug.Log("INFO: ReadAsync string: " + msgJSON.ToString());

                        }

                    } else {
                        Debug.LogWarning("WARNING: 0 bytes read.  connection closed.");
                        _isConnected = false;
                    }

                } catch (System.Exception e) {
                    Debug.LogWarning("ERROR: ZOROSBridgeConnection::ClientReadAsync " + e.ToString());
                }

            }
        }


        public async Task SendJSONStringAsync(string message) {
            if (_isConnected == false) {
                return;
            }

            // all JSON message to or from the server should/will begin with a 0x00 byte and end with 0xFF.
            _tcpClient.GetStream().WriteByte(0x00);
            byte[] messageBytes = Encoding.UTF8.GetBytes(message);
            await _tcpClient.GetStream().WriteAsync(messageBytes, 0, messageBytes.Length);
            _tcpClient.GetStream().WriteByte(0xFF);
        }

        public void SendJSONString(string message) {
            if (_isConnected == false) {
                return;
            }

            // all JSON message to or from the server should/will begin with a 0x00 byte and end with 0xFF.
            _tcpClient.GetStream().WriteByte(0x00);
            byte[] messageBytes = Encoding.UTF8.GetBytes(message);
            _tcpClient.GetStream().Write(messageBytes, 0, messageBytes.Length);
            _tcpClient.GetStream().WriteByte(0xFF);
        }

        public async Task SendBSONAsync(byte[] byteArray) {
            if (_isConnected == false) {
                return;
            }
            if (_tcpClient.Connected == false) {
                _isConnected = false;
                return;
            }
            if (_tcpClient.GetStream() == null) {
                _isConnected = false;
                return;
            }

            try {
                await _tcpClient.GetStream()?.WriteAsync(byteArray, 0, byteArray.Length);
            } catch (Exception e) {
                Debug.LogWarning("WARNING: ZOROSBridgeConnection::SendBSONAsync: " + e.ToString());
            }
        }
        public async Task SendBSONAsync(MemoryStream memoryStream) {
            await SendBSONAsync(memoryStream.ToArray());
        }

        public async Task SendBSONAsync(JObject json) {

            MemoryStream memoryStream = new MemoryStream();
            using (BsonDataWriter writer = new BsonDataWriter(memoryStream)) {
                json.WriteTo(writer);
            }
            await SendBSONAsync(memoryStream);
        }

        public void SendBSON(byte[] byteArray) {
            if (_isConnected == false) {
                Debug.LogWarning("WARNING: SendBSON when not connected.");
                return;
            }

            _tcpClient.GetStream().Write(byteArray, 0, byteArray.Length);
        }

        public void SendBSON(MemoryStream memoryStream) {
            SendBSON(memoryStream.ToArray());
        }

        public void SendBSON(JObject json) {

            MemoryStream memoryStream = new MemoryStream();
            using (BsonDataWriter writer = new BsonDataWriter(memoryStream)) {
                json.WriteTo(writer);
            }
            SendBSON(memoryStream);
        }



    }
}