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
    /// A ROS Spawn Service for .zosim files and Unity prefabs.
    /// </summary>
    public class ZOSimPrefabSpawnService : ZOROSUnityGameObjectBase {

        private Queue<Tuple<ZOSimPrefabSpawnRequest, string>> _spawnZOSimModelRequests = new Queue<Tuple<ZOSimPrefabSpawnRequest, string>>();

        private void Reset() {
            _ROSTopic = "spawn_prefab_model";
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
            // handle any spawn model requests
            while (_spawnZOSimModelRequests.Count > 0) {

                Tuple<ZOSimPrefabSpawnRequest, string> spawnRequestAndId = _spawnZOSimModelRequests.Dequeue();
                ZOSimPrefabSpawnRequest spawnRequest = spawnRequestAndId.Item1;
                string service_request_id = spawnRequestAndId.Item2;

                Debug.Log("INFO: Spawning ZeroSim model: " + spawnRequest.model_name);

                try {
                    Debug.LogError("TODO: Reimplement asset loading with addressables");
                    //GameObject loadedAsset = ROSUnityManager.DefaultAssets.LoadAsset<GameObject>(spawnRequest.model_prefab_name);
                    GameObject loadedAsset = null;
                    if (loadedAsset != null) {
                        Vector3 position = spawnRequest.initial_pose.position.ToUnityVector3();
                        Quaternion rotation = spawnRequest.initial_pose.orientation.ToUnityQuaternion();
                        GameObject instance = Instantiate(loadedAsset, position, rotation);
                        instance.name = spawnRequest.model_name;

                        // report back success
                        ROSBridgeConnection.ServiceResponse<ZOSimPrefabSpawnResponse>(new ZOSimPrefabSpawnResponse() {
                            success = true,
                            status_message = "done!"
                        }, ROSTopic, true, service_request_id);

                    } else { // error loading asset
                        ROSBridgeConnection.ServiceResponse<ZOSimPrefabSpawnResponse>(new ZOSimPrefabSpawnResponse() {
                            success = false,
                            status_message = "ERROR: loading model: " + spawnRequest.model_name
                        }, ROSTopic, false, service_request_id);

                    }


                    // // spawn a new gameobject
                    // GameObject newZoSimModel = new GameObject(spawnRequest.model_name);

                    // newZoSimModel.transform.parent = null;  // make it child of the world

                    // // set the initial position and orientation
                    // Vector3 position = spawnRequest.initial_pose.position.ToUnityVector3();
                    // Quaternion rotation = spawnRequest.initial_pose.orientation.ToUnityQuaternion();
                    // newZoSimModel.transform.position = position;
                    // newZoSimModel.transform.rotation = rotation;

                    // // load from JSON
                    // ZOSimDocumentRoot simDocumentRoot = newZoSimModel.AddComponent<ZOSimDocumentRoot>();
                    // JObject zosimModelJSON = JObject.Parse(spawnRequest.model_zosim);
                    // simDocumentRoot.LoadFromJSON(zosimModelJSON);

                    // // fixup name
                    // newZoSimModel.name = spawnRequest.model_name;

                    // report back success
                    // ROSBridgeConnection.ServiceResponse<ZOSimPrefabSpawnResponse>(new ZOSimPrefabSpawnResponse() {
                    //     success = true,
                    //     status_message = "success!"
                    // }, ROSTopic, true, service_request_id);


                } catch (System.Exception e) {
                    Debug.LogError("ERROR: spawning model: " + e.ToString());

                    // report back error
                    ROSBridgeConnection.ServiceResponse<ZOSimPrefabSpawnResponse>(new ZOSimPrefabSpawnResponse() {
                        success = true,
                        status_message = "ERROR: " + e.ToString()
                    }, ROSTopic, true, service_request_id);

                }



            }

        }

        private void Initialize() {
            Debug.Log("INFO: ZOSimPrefabSpawnService::Initialize");
            // advertise
            ROSBridgeConnection.AdvertiseService<ZOSimPrefabSpawnRequest>(ROSTopic, "zero_sim_ros/ZOSimPrefabSpawn", (rosBridge, msg, id) => {
                Debug.Log("INFO: ZOSimPrefabSpawnService::SpawnModelHandler");

                // queue up the spawn model because it needs to be done in the main thread Update()
                _spawnZOSimModelRequests.Enqueue(new Tuple<ZOSimPrefabSpawnRequest, string>((ZOSimPrefabSpawnRequest)msg, id));


                return Task.CompletedTask;

            });

        }

        // private Task SpawnModelHandler(ZOROSBridgeConnection rosBridgeConnection, ZOROSMessageInterface msg, string id) {
        //     Debug.Log("INFO: ZOROSUnityManager::SpawnModelHandler");

        //     // queue up the spawn model because it needs to be done in the main thread Update()
        //     _spawnZOSimModelRequests.Enqueue(new Tuple<ZOSimSpawnServiceRequest, string>((ZOSimSpawnServiceRequest)msg, id));


        //     return Task.CompletedTask;
        // }


        public override void OnROSBridgeConnected(object rosUnityManager) {
            Debug.Log("INFO: ZOSimSpawnService::OnROSBridgeConnected");
            Initialize();
        }

        public override void OnROSBridgeDisconnected(object rosUnityManager) {
            Debug.Log("INFO: ZOSimSpawnService::OnROSBridgeDisconnected");
            ROSBridgeConnection.UnAdvertise(ROSTopic);
        }

    }

}
