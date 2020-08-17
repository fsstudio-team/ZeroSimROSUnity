using System.Threading;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ZO.ROS;
using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.ROSAPI;
using System.Net.Sockets;

/// <summary>
/// Tests simple ROS Bridge publishing and subscriptions
/// 
/// Launch ROS Bridge: <c>`roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=true`</c>
/// Launch Publisher Test: <c>`rosrun zero_sim_ros zo_publisher_test.py`</c>
/// </summary>
public class ZOROSBridgeUnitTest : MonoBehaviour {

    // Start is called before the first frame update
    void Start() {
        /// # run a ROS Bridge Connection TCP server for BSON encoding:
        /// roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=true
        ZOROSBridgeConnection.Instance.Serialization = ZOROSBridgeConnection.SerializationType.BSON;
        ZOROSBridgeConnection.Instance.ROSBridgeConnectEvent += delegate(ZOROSBridgeConnection rosbridge) {
            Debug.Log("INFO: Connected to ROS Bridge");

            // test advertising
            rosbridge.Advertise("cmd_vel_zero", "geometry_msgs/Twist");

            // test ROS subscription
            // NOTE: this message should alway be returned by the ROS Bridge.  If not there is something wrong
            // with the `roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=true` setup
            rosbridge.Subscribe<Int32Message>("subscriber_1", "client_count", "std_msgs/Int32", (bridge, msg) => {
                Debug.Log("INFO:  Received subscription: " + ((Int32Message)msg).data);
                return Task.CompletedTask;
            });

            // test ROS subscription
            // See: `zero_sim_ros/scripts/zo_publisher_test.py`
            // `rosrun zero_sim_ros zo_publisher_test.py`
            rosbridge.Subscribe<StringMessage>("subscriber_1", "chatter", "std_msgs/String", (bridge, msg) => {
                Debug.Log("INFO:  Received subscription: " + ((StringMessage)msg).data);
                return Task.CompletedTask;
            });

            // test ROS service provider
            // On command line run: `rosservice call /unity/test_service true`
            rosbridge.AdvertiseService<SetBoolServiceRequest>("unity/test_service", "std_srvs/SetBool", (bridge, msg, id) => {
                Debug.Log("INFO: Servicing the unity/test_service request: " + ((SetBoolServiceRequest)msg).data.ToString());

                // respond back
                bridge.ServiceResponse<SetBoolServiceResponse>(new SetBoolServiceResponse(true, "hello world"), "unity/test_service", true, id);

                return Task.CompletedTask;

            });



            // test ROS service call by getting rosparam param names from rosbridge service
            rosbridge.CallService<EmptyServiceRequest, GetParamNamesResponse>(new EmptyServiceRequest(), "/rosapi/get_param_names", "unity_test_1", (bridge, responseMsg) => {
                GetParamNamesResponse response = (GetParamNamesResponse)responseMsg;
                foreach (string name in response.names) {
                    Debug.Log("INFO: Received unity_test_1 rosapi/GetParamNames response name: " + name);
                }

                return Task.CompletedTask;
            });



        };
        ZOROSBridgeConnection.Instance.ROSBridgeDisconnectEvent += delegate(ZOROSBridgeConnection controller) {
            Debug.Log("INFO: Disconnected to ROS Bridge");
            ZOROSBridgeConnection.Instance.UnAdvertiseService("unity/test_service");
            controller.UnAdvertise("cmd_vel_zero");
        };

        // connect to ROS Bridge asynchronously. 
        Task controllerSubscriptionTask = Task.Run(async () => {
            await ZOROSBridgeConnection.Instance.ConnectAsync();
        });
    }

    // Update is called once per frame
    void Update() {
    }

    private void OnDestroy() {


        ZOROSBridgeConnection.Instance.Stop();
    }
}
