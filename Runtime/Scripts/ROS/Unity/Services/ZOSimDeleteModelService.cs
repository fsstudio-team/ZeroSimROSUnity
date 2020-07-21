using System;
using System.IO;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.ZOSim;

namespace ZO.ROS.Unity.Service {

    /// <summary>
    /// A ROS service for deleting a model in the scene.
    /// </summary>
    public class ZOSimDeleteModelService : ZOROSUnityGameObjectBase {

        private Queue<Tuple<ZOSimDeleteModelRequest, string>> _deleteModelRequests = new Queue<Tuple<ZOSimDeleteModelRequest, string>>();

        private void Reset() {
            _ROSTopic = "delete_model";
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
            while (_deleteModelRequests.Count > 0) {

                Tuple<ZOSimDeleteModelRequest, string> deleteRequestAndId = _deleteModelRequests.Dequeue();
                ZOSimDeleteModelRequest deleteRequest = deleteRequestAndId.Item1;
                string service_request_id = deleteRequestAndId.Item2;

                Debug.Log("INFO: Deleting ZeroSim model: " + deleteRequest.model_name);

                try {

                    // find all the zosim document root objects with the model name
                    // BUGBUG: this will not work with prefabs?
                    ZOSimDocumentRoot[] docRoots = UnityEngine.Object.FindObjectsOfType<ZOSimDocumentRoot>();
                    List<ZOSimDocumentRoot> docRootMatches = new List<ZOSimDocumentRoot>();
                    foreach(ZOSimDocumentRoot docRoot in docRoots) {
                        if (docRoot.Name == deleteRequest.model_name) {
                            docRootMatches.Add(docRoot);
                        }
                    }

                    if (docRootMatches.Count > 0) {
                        foreach(ZOSimDocumentRoot docRoot in docRootMatches) {
                            Destroy(docRoot.gameObject);
                        }
                        ROSBridgeConnection.ServiceResponse<ZOSimDeleteModelResponse>(new ZOSimDeleteModelResponse() {
                            success = true,
                            status_message = "INFO: success deleting model: " + deleteRequest.model_name
                        }, ROSTopic, false, service_request_id);

                    } else { // error could not find model to delete
                        ROSBridgeConnection.ServiceResponse<ZOSimDeleteModelResponse>(new ZOSimDeleteModelResponse() {
                            success = false,
                            status_message = "ERROR: deleting model: " + deleteRequest.model_name + " it does not exist in scene."
                        }, ROSTopic, false, service_request_id);
                    }

                } catch (System.Exception e) {
                    Debug.LogError("ERROR: deleting model: " + e.ToString());

                    // report back error
                    ROSBridgeConnection.ServiceResponse<ZOSimDeleteModelResponse>(new ZOSimDeleteModelResponse() {
                        success = true,
                        status_message = "ERROR: " + e.ToString()
                    }, ROSTopic, false, service_request_id);

                }

            }

        }

        private void Initialize() {
            Debug.Log("INFO: ZOSimDeleteModelService::Initialize");
            // advertise
            ROSBridgeConnection.AdvertiseService<ZOSimDeleteModelRequest>(ROSTopic, "zero_sim_ros/ZOSimDeleteModel", (rosBridge, msg, id) => {
                Debug.Log("INFO: ZOSimDeleteModelService::SpawnModelHandler");

                // queue up the spawn model because it needs to be done in the main thread Update()
                _deleteModelRequests.Enqueue(new Tuple<ZOSimDeleteModelRequest, string>((ZOSimDeleteModelRequest)msg, id));

                return Task.CompletedTask;

            });

        }


        public override void OnROSBridgeConnected(object rosUnityManager) {
            Debug.Log("INFO: ZOSimDeleteModelService::OnROSBridgeConnected");
            Initialize();
        }

        public override void OnROSBridgeDisconnected(object rosUnityManager) {
            Debug.Log("INFO: ZOSimDeleteModelService::OnROSBridgeDisconnected");
            ROSBridgeConnection.UnAdvertise(ROSTopic);
        }

    }

}
