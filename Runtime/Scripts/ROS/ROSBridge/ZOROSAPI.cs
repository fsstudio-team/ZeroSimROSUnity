using System;
using System.Threading.Tasks;
using ZO.ROS.MessageTypes.ROSAPI;
using ZO.ROS.MessageTypes.Std;
using Newtonsoft.Json;

namespace ZO.ROS {

    /// <summary>
    /// rosapi provides service calls for getting meta-information related to ROS like 
    /// topic lists as well as interacting with the Parameter Server.
    /// </summary>
    public static class ZOROSAPI {
        /// <summary>
        /// Get all the parameter names published on the rosparam server.
        /// </summary>
        /// <param name="id">Unique ID for the service call.</param>
        /// <param name="onGetParamNames">Callback on getting the parameter names.</param>
        public static void GetParamNames(string id, Func<string[], Task> onGetParamNames) {
            ZOROSBridgeConnection.Instance.CallService<EmptyServiceRequest, GetParamNamesResponse>(new EmptyServiceRequest(), "/rosapi/get_param_names", id, (bridge, responseMsg) => {
                GetParamNamesResponse response = (GetParamNamesResponse)responseMsg;
                onGetParamNames(response.names);
                return Task.CompletedTask;
            });
        }


        /// <summary>
        /// Get a rosparam parameter.
        /// </summary>
        /// <param name="paramName">Parameter name</param>
        /// <param name="id">Unique ID for the service call.</param>
        /// <param name="onGetParam">Callback on getting the parameter value.</param>
        public static void GetParam(string paramName, string id, Func<string, Task> onGetParam) {
            // Debug.Log("INFO: GetParam: " + paramName + " id: " + id);
            GetParamServiceRequest getParamServiceRequest = new GetParamServiceRequest(paramName, "");
            ZOROSBridgeConnection.Instance.CallService<GetParamServiceRequest, GetParamResponse>(getParamServiceRequest, "/rosapi/get_param", id, (bridge, responseMsg) => {
                // Debug.Log("INFO: GetParamResponse: " + id);
                GetParamResponse response = (GetParamResponse)responseMsg;
                onGetParam(response.value);
                return Task.CompletedTask;
            });
        }

        /// <summary>
        /// Set a rosparam parameter.
        /// </summary>
        /// <param name="paramName">Parameter Name</param>
        /// <param name="paramValue">Parameter value</param>
        /// <param name="id">Unique ID for the service call.</param>
        public static void SetParam(string paramName, string paramValue, string id) {
            string serializedParamValue = JsonConvert.SerializeObject(paramValue);
            SetParamServiceRequest setParamServiceRequest = new SetParamServiceRequest(paramName, serializedParamValue);
            ZOROSBridgeConnection.Instance.CallService<SetParamServiceRequest, EmptyServiceRespone>(setParamServiceRequest, "/rosapi/set_param", id, (bridge, responseMsg) => {
                return Task.CompletedTask;
            });
        }



        /// <summary>
        /// Checks if a ROS parameter exists.
        /// </summary>
        /// <param name="paramName">Name of the parameter</param>
        /// <param name="id">Unique id of caller</param>
        /// <param name="onHasParam">callback for result.</param>
        public static void HasParam(string paramName, string id, Func<bool, Task> onHasParam) {
            HasParamServiceRequest hasParamServiceRequest = new HasParamServiceRequest(paramName);
            ZOROSBridgeConnection.Instance.CallService<HasParamServiceRequest, HasParamResponse>(hasParamServiceRequest, "/rosapi/has_param", id, (bridge, responseMsg) => {
                HasParamResponse response = (HasParamResponse)responseMsg;
                onHasParam(response.exists);
                return Task.CompletedTask;
            });

        }


    }
}