using System.Linq;
using System;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.ControllerManager;
using ZO.ROS.Controllers;
using ZO.Document;

namespace ZO.ROS.Unity.Service {

    /// <summary>
    /// Provides `controller_manager` services.
    /// 
    /// * load: load controllers (construct and initialize)
    /// * unload: unload controllers (destruct)
    /// * start: start controllers
    /// * stop: stop controllers
    /// * spawn: load and start controllers
    /// * kill: stop and unload controllers 
    /// 
    /// <see>http://wiki.ros.org/controller_manager</see>
    /// </summary>
    public class ZOControllerManagerService : ZOROSUnityGameObjectBase {


        private Dictionary<string, ZOROSControllerInterface> _controllers = new Dictionary<string, ZOROSControllerInterface>();
        public void RegisterController(ZOROSControllerInterface controller) {
            _controllers.Add(controller.ControllerName, controller);
        }



        public string ListControllersServiceTopic {
            get => Name + "/controller_manager/list_controllers";
        }

        public string LoadControllerServiceTopic {
            get => Name + "/controller_manager/load_controller";
        }

        public string UnloadControllerServiceTopic {
            get => Name + "/controller_manager/unload_controller";
        }

        public string ReloadControllerServiceTopic {
            get => Name + "/controller_manager/reload_controller_libraries";
        }

        public string SwitchControllerServiceTopic {
            get => Name + "/controller_manager/switch_controller";
        }

        public string ListControllerTypesServiceTopic {
            get => Name + "/controller_manager/list_controller_types";
        }

        protected override void ZOReset() {
            base.ZOReset();
            _ROSTopic = "controller_manager";
            UpdateRateHz = 5.0f;
        }



        #region ZOGameObjectBase

        protected override void ZOAwake() {
            base.ZOAwake();

        }

        protected override void ZOStart() {
            base.ZOStart();
            if (ROSBridgeConnection.IsConnected) {
                Initialize();
            }
        }
        protected override void ZOOnDestroy() {
            base.ZOOnDestroy();
            Terminate();
        }



        protected override void ZOUpdate() {

        }
        #endregion // ZOGameObjectBase

        private void Initialize() {
            Debug.Log("INFO: ZOControllerManagerService::Initialize");

            // advertise list controllers
            ROSBridgeConnection.AdvertiseService<EmptyServiceRequest>(ListControllersServiceTopic,
                ListControllersResponse.Type,
                (rosBridge, msg, id) => {

                    // build a list of all the registered controllers
                    List<ControllerStateMessage> controllerStateMessages = new List<ControllerStateMessage>();
                    foreach (KeyValuePair<string, ZOROSControllerInterface> entry in _controllers) {
                        controllerStateMessages.Add(entry.Value.ControllerStateMessage);
                    }
                    // Debug.Log("INFO: ZOControllerManagerService::ListControllersService: " + string.Join(",", controllerStateMessages.ToString()));

                    ROSBridgeConnection.ServiceResponse<ListControllersResponse>(new ListControllersResponse(controllerStateMessages.ToArray()), ListControllersServiceTopic, true, id);



                    return Task.CompletedTask;

                });

            // advertise list types controllers
            ROSBridgeConnection.AdvertiseService<EmptyServiceRequest>(ListControllerTypesServiceTopic,
                ListControllerTypesServiceResponse.Type,
                (rosBridge, msg, id) => {
                    Debug.Log("INFO: ZOControllerManagerService::ListControllerTypesService");

                    // build a list of all the registered controllers
                    List<string> types = new List<string>();
                    List<string> base_classes = new List<string>();
                    foreach (KeyValuePair<string, ZOROSControllerInterface> entry in _controllers) {
                        types.Add(entry.Value.ControllerType);
                        base_classes.Add("JointPositionController"); //HACKHACK: hardwirds
                    }
                    ROSBridgeConnection.ServiceResponse<ListControllerTypesServiceResponse>(new ListControllerTypesServiceResponse(types.ToArray(), base_classes.ToArray()), ListControllerTypesServiceTopic, true, id);


                    return Task.CompletedTask;

                });


            // advertise load controllers
            ROSBridgeConnection.AdvertiseService<LoadControllerServiceRequest>(LoadControllerServiceTopic,
                LoadControllerServiceRequest.Type,
                (rosBridge, msg, id) => {
                    LoadControllerServiceRequest loadControllerServiceRequest = msg as LoadControllerServiceRequest;

                    Debug.Log("INFO: ZOControllerManagerService::LoadControllerServiceRequest: "
                                + loadControllerServiceRequest.name);


                    // check if controller is registered
                    if (_controllers.ContainsKey(loadControllerServiceRequest.name)) {

                        // initialize the actual controller
                        _controllers[loadControllerServiceRequest.name].LoadController();

                        // respond back we loaded the controller succesfully
                        ROSBridgeConnection.ServiceResponse<LoadControllerServiceResponse>(new LoadControllerServiceResponse(true), LoadControllerServiceTopic, true, id);

                    } else {
                        Debug.LogWarning("WARNING: could not load controller: " + loadControllerServiceRequest.name);
                        // controller is not registered so respond back an error
                        ROSBridgeConnection.ServiceResponse<LoadControllerServiceResponse>(new LoadControllerServiceResponse(false), LoadControllerServiceTopic, false, id);

                    }


                    return Task.CompletedTask;

                });

            // advertise switch controllers
            ROSBridgeConnection.AdvertiseService<SwitchControllerServiceRequest>(SwitchControllerServiceTopic,
                SwitchControllerServiceRequest.Type,
                (rosBridge, msg, id) => {
                    SwitchControllerServiceRequest switchControllerServiceRequest = msg as SwitchControllerServiceRequest;


                    bool success = true;
                    foreach (string controller in switchControllerServiceRequest.stop_controllers) {

                        Debug.Log("INFO: ZOControllerManagerService::SwitchControllerServiceRequest: Stop Controller: " + controller);

                        // check if controller is registered
                        if (_controllers.ContainsKey(controller)) {

                            // stop the controller if necessary
                            _controllers[controller].StopController();

                        } else {
                            Debug.LogWarning("WARNING: could not stop controller: " + controller);
                            success = false;
                        }

                    }

                    foreach (string controller in switchControllerServiceRequest.start_controllers) {

                        Debug.Log("INFO: ZOControllerManagerService::SwitchControllerServiceRequest: Start Controller: " + controller);

                        // check if controller is registered
                        if (_controllers.ContainsKey(controller)) {

                            // initialize the actual controller
                            _controllers[controller].StartController();

                        } else {
                            Debug.LogWarning("WARNING: could not start controller: " + controller);
                            success = false;
                        }

                    }

                    // controller is not registered so respond back an error
                    ROSBridgeConnection.ServiceResponse<SwitchControllerServiceResponse>(new SwitchControllerServiceResponse(success), SwitchControllerServiceTopic, success, id);



                    return Task.CompletedTask;

                });

            // advertise unload service
            ROSBridgeConnection.AdvertiseService<UnloadControllerServiceRequest>(UnloadControllerServiceTopic,
                UnloadControllerServiceRequest.Type,
                (rosBridge, msg, id) => {
                    UnloadControllerServiceRequest unloadControllerServiceRequest = msg as UnloadControllerServiceRequest;

                    Debug.Log("INFO: ZOControllerManagerService::UnloadControllerServiceRequest: "
                                + unloadControllerServiceRequest.name);


                    // check if controller is registered
                    if (_controllers.ContainsKey(unloadControllerServiceRequest.name)) {

                        // initialize the actual controller
                        _controllers[unloadControllerServiceRequest.name].UnloadController();

                        // respond back we loaded the controller succesfully
                        ROSBridgeConnection.ServiceResponse<UnloadControllerServiceResponse>(new UnloadControllerServiceResponse(true), UnloadControllerServiceTopic, true, id);

                    } else {
                        Debug.LogWarning("WARNING: could not unload controller: " + unloadControllerServiceRequest.name);
                        // controller is not registered so respond back an error
                        ROSBridgeConnection.ServiceResponse<UnloadControllerServiceResponse>(new UnloadControllerServiceResponse(false), UnloadControllerServiceTopic, false, id);

                    }


                    return Task.CompletedTask;

                });

            // reload
            ROSBridgeConnection.AdvertiseService<ReloadControllerLibrariesServiceRequest>(ReloadControllerServiceTopic,
                ReloadControllerLibrariesServiceRequest.Type,
                (rosBridge, msg, id) => {
                    ReloadControllerLibrariesServiceRequest unloadControllerServiceRequest = msg as ReloadControllerLibrariesServiceRequest;

                    Debug.Log("INFO: ZOControllerManagerService::ReloadControllerServiceRequest");

                    foreach (KeyValuePair<string, ZOROSControllerInterface> entry in _controllers) {
                        entry.Value.UnloadController();
                        entry.Value.LoadController();
                    }


                    return Task.CompletedTask;

                });

        }

        private void Terminate() {
            Debug.Log("INFO: ZOControllerManagerService::Terminate");
            ROSBridgeConnection.UnAdvertiseService(ListControllersServiceTopic);
            ROSBridgeConnection.UnAdvertiseService(LoadControllerServiceTopic);
            ROSBridgeConnection.UnAdvertiseService(SwitchControllerServiceTopic);
            ROSBridgeConnection.UnAdvertiseService(UnloadControllerServiceTopic);
            ROSBridgeConnection.UnAdvertiseService(ListControllerTypesServiceTopic);
            ROSBridgeConnection.UnAdvertiseService(ReloadControllerServiceTopic);
        }


        #region ZOROSUnityInterface
        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOControllerManagerService::OnROSBridgeConnected");
            Initialize();
        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOControllerManagerService::OnROSBridgeDisconnected");
            Terminate();
        }

        #endregion

        #region ZOSerializationInterface

        public string Type {
            get => "controller.controller_manager";
        }

        public override JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("ros_topic", ROSTopic),
                new JProperty("update_rate_hz", UpdateRateHz)
            );
            JSON = json;
            return json;
        }

        public override void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            Name = json["name"].Value<string>();
            UpdateRateHz = json["update_rate_hz"].Value<float>();
            ROSTopic = json["ros_topic"].Value<string>();
        }

        #endregion // ZOSerializationInterface

    }

}
