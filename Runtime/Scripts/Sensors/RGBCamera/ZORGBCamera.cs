using System.IO;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.Util;
using ZO.Util.Extensions;
using ZO.ROS.Unity;

namespace ZO.Sensors {
    [RequireComponent(typeof(Camera))]
    public class ZORGBCamera : ZOGameObjectBase, ZOSerializationInterface {
        [Header("Camera Parameters")]
        // string _cameraId = "none";
        public Camera _camera;
        public Camera UnityCamera {
            get => _camera;
            private set => _camera = value;
        }
        public int _width = 1280;
        public int _height = 720;

        [Header("Render Parameters")]
        [SerializeField]
        public Material _postProcessMaterial;

        [Header("Debug")]
        public Vector2 _debugWindowPosition = new Vector2(10, 10);
        private Texture2D _debugTexture;

        // ~~~~~~ Delegate Callbacks ~~~~~~
        /// <summary>
        /// Called every frame passing in:
        /// this, string cameraId, width, height, RGB24[] image
        /// 
        /// Note: is async so returns a task
        /// </summary>
        /// 
        /// <value></value>
        public Func<ZORGBCamera, string, int, int, byte[], Task> OnPublishRGBImageDelegate { get; set; }

        byte[] _colorPixels24;
        Task _publishTask = null;

        // ~~~~~~ Render Buffers ~~~~~~~
        private RenderTexture _renderTexture;

        // ~~~~ Async GPU Read ~~~~ //
        Queue<ZO.Util.Rendering.ZOAsyncGPUReadbackPluginRequest> _requests = new Queue<ZO.Util.Rendering.ZOAsyncGPUReadbackPluginRequest>();


        // Start is called before the first frame update
        protected override void ZOStart() {
            Initialize();
        }

        protected void Initialize() {
            // if camera is not assigned see if we have a camera component on this game object
            if (_camera == null) {
                _camera = this.GetComponent<Camera>();
            }

            if (_postProcessMaterial == null) {
                _postProcessMaterial = ZOROSUnityManager.Instance.DefaultAssets.LoadAsset<Material>("ZORGBPostProcessMaterial");
            }

            _renderTexture = new RenderTexture(_width, _height, 16, RenderTextureFormat.ARGB32);
            _camera.targetTexture = _renderTexture;


            _colorPixels24 = new byte[_width * _height * 3];

            if (IsDebug == true) {
                _debugTexture = new Texture2D(_width, _height, TextureFormat.RGB24, false);
            }


            if (SystemInfo.supportsAsyncGPUReadback == true) {
                Debug.Log("INFO: Native support for AsyncGPUReadback");
            } else {
                Debug.LogWarning("WARNING: NO support for native AsyncGPUReadback. Using 3rd party.");
            }

        }

        protected override void ZOFixedUpdateHzSynchronized() {
            _camera.Render();
        }


        protected override void ZOUpdate() {
            DoRenderTextureUpdate();
        }


        void OnRenderImage(RenderTexture source, RenderTexture destination) {


            Graphics.Blit(source, destination, _postProcessMaterial);
            if (_requests.Count < 8) {
                _requests.Enqueue(ZO.Util.Rendering.ZOAsyncGPUReadbackPlugin.Request(destination));
            } else {
                Debug.LogWarning("INFO: ZORGBCamera::OnRenderImage Too many requests.");
            }


            if (IsDebug == true) {
                RenderTexture.active = destination;
                _debugTexture.ReadPixels(new Rect(0, 0, destination.width, destination.height), 0, 0);
                _debugTexture.Apply();
            }
        }

        private void DoRenderTextureUpdate() {
            // ~~~ Handle Async GPU Readback ~~~ //
            while (_requests.Count > 0) {
                var req = _requests.Peek();
                UnityEngine.Profiling.Profiler.BeginSample("GL Update");
                // You need to explicitly ask for an update regularly
                req.Update();
                UnityEngine.Profiling.Profiler.EndSample();

                if (req.hasError) {
                    Debug.LogError("ERROR: GPU readback error detected.");
                    req.Dispose();
                    _requests.Dequeue();
                } else if (req.done) {
                    UnityEngine.Profiling.Profiler.BeginSample("GetRawData");
                    // Get data from the request when it's done
                    byte[] rawTextureData = req.GetRawData();
                    UnityEngine.Profiling.Profiler.EndSample();
                    if (OnPublishRGBImageDelegate != null) {
                        if (_publishTask == null || _publishTask.IsCompleted) {
                            UnityEngine.Profiling.Profiler.BeginSample("_publishTask");
                            for (int i = 0, c3 = 0, c4 = 0; i < _width * _height; i++, c3 += 3, c4 += 4) {
                                _colorPixels24[c3 + 0] = (byte)(rawTextureData[c4 + 1]);
                                _colorPixels24[c3 + 1] = (byte)(rawTextureData[c4 + 2]);
                                _colorPixels24[c3 + 2] = (byte)(rawTextureData[c4 + 3]);

                            }
                            OnPublishRGBImageDelegate(this, Name, _width, _height, _colorPixels24);
                            UnityEngine.Profiling.Profiler.EndSample();
                        } else {
                            // Debug.Log("skip");
                        }

                    }
                    UnityEngine.Profiling.Profiler.BeginSample("Dispose");
                    // You need to explicitly Dispose data after using them
                    req.Dispose();

                    _requests.Dequeue();
                    UnityEngine.Profiling.Profiler.EndSample();
                } else {
                    break;
                }
            }

        }

        private void OnGUI() {
            if (IsDebug == true) {

                if (_debugTexture) {
                    GUI.DrawTexture(new UnityEngine.Rect(_debugWindowPosition.x, _debugWindowPosition.y, _debugTexture.width, _debugTexture.height), _debugTexture, ScaleMode.ScaleToFit);
                }
            }
        }

#region ZOSerializationInterface
        public string Type {
            get { return "sensor.rgbcamera"; }
        }

        [SerializeField] public string _name;
        public string Name {
            get {
                return _name;
            }
            private set {
                _name = value;
            }
        }

        private JObject _json;
        public JObject JSON {
            get => _json;
        }


        public JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("width", _width),
                new JProperty("height", _height),
                new JProperty("aspect_ratio", UnityCamera.aspect),
                new JProperty("field_of_view", UnityCamera.fieldOfView),
                new JProperty("depth", UnityCamera.depth),
                new JProperty("sensor_size", UnityCamera.sensorSize.ToJSON()),
                new JProperty("focal_length", UnityCamera.focalLength)
            );


            ZOSimOccurrence parent_occurrence = GetComponent<ZOSimOccurrence>();
            if (parent_occurrence) {
                json["parent_occurrence"] = parent_occurrence.Name;
            }

            _json = json;

            return json;
        }

        public void ImportZeroSim(ZOSimDocumentRoot documentRoot, JObject json) {
            throw new System.NotImplementedException("TODO!");
            // TODO:
        }

        public void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            // Assert.Equals(json["type"].Value<string>() == Type);

            if (UnityCamera == null) {  // check if there is already a camera assigned.
                UnityCamera = GetComponent<Camera>();  // check if we have a camera component but it is unassigend
                if (UnityCamera == null) {
                    // we have no camera so create one
                    UnityCamera = gameObject.AddComponent<Camera>();
                }
            }
            _json = json;
            Name = json.ValueOrDefault("name", Name);
            _width = json.ValueOrDefault("width", _width);
            _height = json.ValueOrDefault("height", _height);
            UnityCamera.aspect = json.ValueOrDefault("aspect_ratio", UnityCamera.aspect);
            UnityCamera.fieldOfView = json.ValueOrDefault("field_of_view", UnityCamera.fieldOfView);
            UnityCamera.depth = json.ValueOrDefault("depth", UnityCamera.depth);
            UnityCamera.sensorSize = json.ToVector2OrDefault("sensor_size", UnityCamera.sensorSize);
            UnityCamera.focalLength = json.ValueOrDefault("focal_length", UnityCamera.focalLength);   

            Initialize();

        }
#endregion



    }
}