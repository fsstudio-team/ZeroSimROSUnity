using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;

namespace ZO {

    /// <summary>
    /// DEPRECATED!!!!  DO NOT USE!!! 
    /// </summary>
    public class ZOSceneManager : MonoBehaviour {

        private ZO.Networking.ZOJSONMessagePublisher _runtimeStatusUpdatePublisher;
        private ZO.Networking.ZOJSONMessagePublisher _systemInfoPublisher;

        public class ZORuntimeStatus {
            public float OverallUpdateRateHz {get; set; }
            public float PhysicsUpdateRateHz {get; set; }
        };

        public class ZOSystemInfo {
            public bool SupportsAsyncGPUReadback {get { return SystemInfo.supportsAsyncGPUReadback; } }
            public string DeviceModel {get { return SystemInfo.deviceModel; } }
            public string DeviceName {get { return SystemInfo.deviceName; }}
            public int SystemMemorySize {get { return SystemInfo.systemMemorySize; }}
            public int ProcessorCount {get { return SystemInfo.processorCount; }}
            public int ProcessorFrequency {get { return SystemInfo.processorFrequency; }}
            public string ProcessorType {get { return SystemInfo.processorType; } }
            public string OperatingSystem {get { return SystemInfo.operatingSystem; } }
            public string GraphicsDeviceName {get { return SystemInfo.graphicsDeviceName; }}
            public int GraphicsMemorySize {get { return SystemInfo.graphicsMemorySize; }}
            public string GraphicsDeviceVersion {get { return SystemInfo.graphicsDeviceVersion; }}
            public string GraphicsDeviceVendor {get {return SystemInfo.graphicsDeviceVendor;}}
        };

        private ZOSystemInfo _systemInfo;

        // Start is called before the first frame update
        void Start() {

            // start up the runtime status message publisher
            _runtimeStatusUpdatePublisher = new Networking.ZOJSONMessagePublisher() {
                Topic = "SceneStatusUpdate",
                Port = 4444
            };

            Task.Run(async ()=> {
                await _runtimeStatusUpdatePublisher.RunAsync();
            });

            _systemInfo = new ZOSystemInfo();

            _systemInfoPublisher = new Networking.ZOJSONMessagePublisher() {
                Topic = "SystemInfo",
                Port = 4445
            };

            Task.Run(async ()=> {
                await _systemInfoPublisher.RunAsync();
            });

        }

        // Update is called once per frame
        void Update() {
            if (_runtimeStatusUpdatePublisher.IsSubscribersConnected) {
                ZORuntimeStatus runtimeStatus = new ZORuntimeStatus {
                    OverallUpdateRateHz = 1.0f/Time.smoothDeltaTime,
                    PhysicsUpdateRateHz = 1.0f/Time.fixedDeltaTime
                };
                _runtimeStatusUpdatePublisher.Publish(JsonConvert.SerializeObject(runtimeStatus));
            }

            if (_systemInfoPublisher.IsSubscribersConnected) {
                _systemInfoPublisher.Publish(JsonConvert.SerializeObject(_systemInfo));
            }
        }
    }

}
