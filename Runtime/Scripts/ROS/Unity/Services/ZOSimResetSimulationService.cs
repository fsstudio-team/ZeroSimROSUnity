using System;
using System.IO;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using ZO.ROS.MessageTypes.Std;

namespace ZO.ROS.Unity.Service {

    /// <summary>
    /// A ROS service for deleting a model in the scene.
    /// </summary>
    public class ZOSimResetSimulationService : ZOROSUnityGameObjectBase {

        private Queue<Tuple<EmptyServiceRequest, string>> _resetSimulationRequest = new Queue<Tuple<EmptyServiceRequest, string>>();

        private void Reset() {
            _ROSTopic = "reset_simulation";
            UpdateRateHz = 5.0f;
        }

        protected override void ZOStart() {
            base.ZOStart();
            if (ROSBridgeConnection.IsConnected) {
                Initialize();
            }
        }
        protected override void ZOOnDestroy() {
            ROSBridgeConnection?.UnAdvertise(ROSTopic);
        }

        protected override void ZOUpdate() {
            // handle any delete model requests
            while (_resetSimulationRequest.Count > 0) {

                Tuple<EmptyServiceRequest, string> deleteRequestAndId = _resetSimulationRequest.Dequeue();
                EmptyServiceRequest deleteRequest = deleteRequestAndId.Item1;
                string service_request_id = deleteRequestAndId.Item2;

                Debug.Log("INFO: reseting simulation");

                try {

                    SceneManager.LoadScene (SceneManager.GetActiveScene ().name);
                } catch (System.Exception e) {
                    Debug.LogError("ERROR: reseting simulation: " + e.ToString());

                    // report back error
                    ROSBridgeConnection.ServiceResponse<EmptyServiceRespone>(new EmptyServiceRespone(), ROSTopic, false, service_request_id);

                }

            }

        }

        private void Initialize() {
            Debug.Log("INFO: ZOSimResetSimulationService::Initialize");
            // advertise
            ROSBridgeConnection.AdvertiseService<EmptyServiceRequest>(ROSTopic, "std_srvs/Empty", (rosBridge, msg, id) => {
                Debug.Log("INFO: ZOSimResetSimulationService service request");

                // queue up the spawn model because it needs to be done in the main thread Update()
                _resetSimulationRequest.Enqueue(new Tuple<EmptyServiceRequest, string>((EmptyServiceRequest)msg, id));

                return Task.CompletedTask;

            });

        }


        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOSimResetSimulationService::OnROSBridgeConnected");
            Initialize();
        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOSimResetSimulationService::OnROSBridgeDisconnected");
            ROSBridgeConnection.UnAdvertise(ROSTopic);
        }

    }

}
