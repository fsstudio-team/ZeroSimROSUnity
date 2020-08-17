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
public class ZOROSAPIUnitTest : MonoBehaviour {

    // Start is called before the first frame update
    void Start() {
        /// # run a ROS Bridge Connection TCP server for BSON encoding:
        /// roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=true
        ZOROSBridgeConnection.Instance.Serialization = ZOROSBridgeConnection.SerializationType.BSON;
        ZOROSBridgeConnection.Instance.ROSBridgeConnectEvent += delegate(ZOROSBridgeConnection rosbridge) {
            Debug.Log("INFO: Connected to ROS Bridge");

            // test advertising
            // test getting ROS param names 
            ZOROSAPI.GetParamNames("unity_test_2", (param_names) => {
                foreach (string name in param_names) {
                    Debug.Log("INFO: Received unity_test_2 GetParamNames: " + name);
                }
                return Task.CompletedTask;
            });

            // test getting ROS parameter
            ZOROSAPI.GetParam("/rosdistro", "unity_test_3", (param_value) => {
                Debug.Log("INFO: Test GetParam /rosdistro: " + param_value);
                return Task.CompletedTask;
            });

            // test setting ROS parameter
            ZOROSAPI.SetParam("/test_param_set_string", "Hello World Set Param Test", "unity_test_4");

            // test getting a set parameter
            ZOROSAPI.GetParam("/test_param_set_string", "unity_test_5", (param_value) => {
                Debug.Log("INFO: Test GetParam /test_param_set_string: " + param_value);
                return Task.CompletedTask;
            });

            // test if a parameter exists
            ZOROSAPI.HasParam("/rosdistro", "unity_test_5", (exists) => {
                Debug.Log("INFO: ROS parameter /rosdistro exists: " + exists.ToString());
                return Task.CompletedTask;
            });

            // test if a parameter exists
            ZOROSAPI.HasParam("/random_non_exist_param", "unity_test_6", (exists) => {
                Debug.Log("INFO: ROS parameter /random_non_exist_param exists: " + exists.ToString());
                return Task.CompletedTask;
            });

        };
        ZOROSBridgeConnection.Instance.ROSBridgeDisconnectEvent += delegate(ZOROSBridgeConnection controller) {
            Debug.Log("INFO: Disconnected to ROS Bridge");
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
