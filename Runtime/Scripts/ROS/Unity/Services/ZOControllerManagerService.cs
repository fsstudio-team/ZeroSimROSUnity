using System;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.ControllerManager;

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

        #region Singleton

        private static ZOControllerManagerService _instance;

        /// <summary>
        /// Singleton access to this ROS Unity Manager.
        /// </summary>
        public static ZOControllerManagerService Instance {
            get => _instance;
        }


        #endregion // Singleton


        public string ListControllersServiceTopic {
            get {
                return "/controller_manager/list_controllers";
            }
        }

        private void Reset() {
            _ROSTopic = "controller_manager";
            UpdateRateHz = 5.0f;
        }



        #region ZOGameObjectBase

        protected override void ZOAwake() {
            base.ZOAwake();

            // build singleton instance if it doesn't exist
            if (_instance == null) {
                _instance = this;
                // DontDestroyOnLoad(this.gameObject);
            } else if (_instance != this) {
                Debug.LogError("ERROR: Cannot have two ZOControllerManagerService's!!!");
                Destroy(this.gameObject);
            }
        }

        protected override void ZOStart() {
            base.ZOStart();
            if (ROSBridgeConnection.IsConnected) {
                Initialize();
            }
        }
        protected override void ZOOnDestroy() {
            ROSBridgeConnection?.UnAdvertiseService(ListControllersServiceTopic);
        }



        protected override void ZOUpdate() {

        }
        #endregion // ZOGameObjectBase

        private void Initialize() {
            Debug.Log("INFO: ZOControllerManagerService::Initialize");
            // advertise
            ROSBridgeConnection.AdvertiseService<EmptyServiceRequest>(ListControllersServiceTopic, ListControllersResponse.Type, (rosBridge, msg, id) => {
                Debug.Log("INFO: ZOControllerManagerService::ListControllersService");

                // report back
                // KILLME THIS IS JUST A TEST DUMMY CONTROLLER
                ControllerStateMessage[] controllers = new ControllerStateMessage[1] {
                    new ControllerStateMessage {
                        name = "arm_controller",
                        state = "running",
                        type = "position_controllers/JointTrajectoryController",
                        claimed_resources = new HardwareInterfaceResourcesMessage[1] {
                            new HardwareInterfaceResourcesMessage {
                                hardware_interface = "hardware_interface::JointStateInterface",
                                resources = new string[6] {
                                        "elbow_joint",
                                        "shoulder_lift_joint",
                                        "shoulder_pan_joint",
                                        "wrist_1_joint",
                                        "wrist_2_joint",
                                        "wrist_3_joint"
                                }
                            }
                        }
                    }
                };

                ROSBridgeConnection.ServiceResponse<ListControllersResponse>(new ListControllersResponse(controllers), ListControllersServiceTopic, true, id);


                return Task.CompletedTask;

            });

        }


        #region ZOROSUnityInterface
        public override void OnROSBridgeConnected(object rosUnityManager) {
            Debug.Log("INFO: ZOControllerManagerService::OnROSBridgeConnected");
            Initialize();
        }

        public override void OnROSBridgeDisconnected(object rosUnityManager) {
            Debug.Log("INFO: ZOControllerManagerService::OnROSBridgeDisconnected");
            ROSBridgeConnection.UnAdvertiseService(ListControllersServiceTopic);
        }

        #endregion
    }

}
