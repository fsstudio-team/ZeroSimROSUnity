using System.IO;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using ZO.Util;

namespace ZO.Sensors {
    [RequireComponent(typeof(Camera))]
    public class ZORGBCamera : ZOGameObjectBase {
        [Header("Camera Parameters")]
        string _cameraId = "none";
        public Camera _camera;
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
            if (_camera == null) {
                _camera = this.GetComponent<Camera>();
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
                Debug.Log("WARNING: NO support for native AsyncGPUReadback. Using 3rd party.");
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
                            OnPublishRGBImageDelegate(this, _cameraId, _width, _height, _colorPixels24);
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


    }
}