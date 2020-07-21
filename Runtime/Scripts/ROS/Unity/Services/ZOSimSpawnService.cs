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
    public class ZOSimSpawnService : ZOROSUnityGameObjectBase {

        private Queue<Tuple<ZOSimSpawnServiceRequest, string>> _spawnZOSimModelRequests = new Queue<Tuple<ZOSimSpawnServiceRequest, string>>();

        private void Reset() {
            _ROSTopic = "spawn_zosim_model";
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

                Tuple<ZOSimSpawnServiceRequest, string> spawnRequestAndId = _spawnZOSimModelRequests.Dequeue();
                ZOSimSpawnServiceRequest spawnRequest = spawnRequestAndId.Item1;
                string service_request_id = spawnRequestAndId.Item2;

                Debug.Log("INFO: Spawning ZeroSim model: " + spawnRequest.model_name);

                try {
                    // spawn a new gameobject
                    GameObject newZoSimModel = new GameObject(spawnRequest.model_name);

                    newZoSimModel.transform.parent = null;  // make it child of the world

                    // set the initial position and orientation
                    Vector3 position = spawnRequest.initial_pose.position.ToUnityVector3();
                    Debug.Log("INFO: Spawning at location: " + position.ToString());
                    Quaternion rotation = spawnRequest.initial_pose.orientation.ToUnityQuaternion();
                    newZoSimModel.transform.position = position;
                    newZoSimModel.transform.rotation = rotation;

                    // load from JSON
                    ZOSimDocumentRoot simDocumentRoot = newZoSimModel.AddComponent<ZOSimDocumentRoot>();
                    JObject zosimModelJSON = JObject.Parse(spawnRequest.model_zosim);
                    simDocumentRoot.Deserialize(zosimModelJSON);

                    // fixup name
                    // TODO: check that the name is unique and if not generate a unique name or maybe return a
                    // warning that the name is not unique?  probably the later.
                    simDocumentRoot.Name = spawnRequest.model_name;

                    // report back success
                    ROSBridgeConnection.ServiceResponse<ZOSimSpawnServiceResponse>(new ZOSimSpawnServiceResponse() {
                        success = true,
                        status_message = "success!"
                    }, ROSTopic, true, service_request_id);


                } catch (System.Exception e) {
                    Debug.LogError("ERROR: spawning model: " + e.ToString());

                    // report back error
                    ROSBridgeConnection.ServiceResponse<ZOSimSpawnServiceResponse>(new ZOSimSpawnServiceResponse() {
                        success = true,
                        status_message = "ERROR: " + e.ToString()
                    }, ROSTopic, true, service_request_id);

                }


                // GameObject loadedAsset = DefaultAssets.LoadAsset<GameObject>(spawnRequest.model_name);
                // if (loadedAsset != null) {
                //     Vector3 position = spawnRequest.initial_pose.position.ToUnityVector3();
                //     Quaternion rotation = spawnRequest.initial_pose.orientation.ToUnityQuaternion();
                //     Instantiate(loadedAsset, position, rotation);

                //     // report back success
                //     ROSBridgeConnection.ServiceResponse<ZOSimSpawnServiceResponse>(new ZOSimSpawnServiceResponse() {
                //         success = true,
                //         status_message = "done!"
                //     }, "gazebo/spawn_urdf_model", true, id);

                // } else { // error loading asset
                //     ROSBridgeConnection.ServiceResponse<ZOSimSpawnServiceResponse>(new ZOSimSpawnServiceResponse() {
                //         success = false,
                //         status_message = "ERROR: loading model: " + spawnRequest.model_name
                //     }, "gazebo/spawn_urdf_model", false, id);

                // }

            }

        }

        private void Initialize() {
            Debug.Log("INFO: ZOSimSpawnService::Initialize");
            // advertise
            ROSBridgeConnection.AdvertiseService<ZOSimSpawnServiceRequest>(ROSTopic, "zero_sim_ros/ZOSimSpawn", (rosBridge, msg, id) => {
                Debug.Log("INFO: ZOSimSpawnService::SpawnModelHandler");

                // queue up the spawn model because it needs to be done in the main thread Update()
                _spawnZOSimModelRequests.Enqueue(new Tuple<ZOSimSpawnServiceRequest, string>((ZOSimSpawnServiceRequest)msg, id));


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
