using System.IO;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using AsyncGPUReadbackPluginNs;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using ZO.Util;

namespace ZO.Sensors {

    /// <summary>
    /// This job process the framebuffer data of float32 R, G, B, D pixels and converts to 
    /// </summary>
    [BurstCompile(CompileSynchronously = true)]
    struct ZOPostProcessFrameBuffer : IJobParallelForBatch {
        [ReadOnly] public float DepthScale;
        [ReadOnly] public NativeArray<float> FrameBuffer;

        public NativeArray<float> DepthValues;
        public NativeArray<byte> RGBValues;
        public void Execute(int startIndex, int count) {
            int end = startIndex + count;
            int pixelIdx = startIndex / 4;
            int rgbPixelIdx = pixelIdx * 3;
            float r = FrameBuffer[startIndex + 0];
            float g = FrameBuffer[startIndex + 0];
            float b = FrameBuffer[startIndex + 0];
            float d = FrameBuffer[startIndex + 0];

            RGBValues[rgbPixelIdx + 0] = (byte)(b * 255.0f);
            RGBValues[rgbPixelIdx + 1] = (byte)(g * 255.0f);
            RGBValues[rgbPixelIdx + 2] = (byte)(r * 255.0f);
            DepthValues[pixelIdx] = d * DepthScale;


        }
    };


    /// <summary>
    /// A RGB image + depth camera sensor. 
    /// </summary>
    public class ZORGBDepthCamera : ZOGameObjectBase {

        public enum FrameOutputType {
            RGB8D32,
            RGB8D16
        };

        [Header("Camera Parameters")]
        public string _cameraId = "none";

        /// <summary>
        /// The unique name identifying this depth camera.
        /// </summary>
        /// <value></value>
        public string Name {
            get => _cameraId;
        }
        public Camera _camera;

        /// <summary>
        /// The Unity Camera associated with this depth camera. (readonly)
        /// </summary>
        /// <value></value>
        public Camera UnityCamera {
            get => _camera;
        }

        public int _width = 1280;
        
        /// <summary>
        /// Camera frame width in pixels.  (readonly)
        /// </summary>
        /// <value></value>
        public int Width {
            get => _width;
        }

        public int _height = 720;

        /// <summary>
        /// Camera frame height in pixels. (readonly)
        /// </summary>
        /// <value></value>
        public int Height {
            get => _height;
        }

        /// <summary>
        /// The cameras field of view in degrees.
        /// </summary>
        /// <value></value>
        public float FieldOfViewDegrees {
            get {
                return UnityCamera.fieldOfView;
            }
        }

        //public FrameOutputType _frameOutputType = FrameOutputType.RGB8D16;

        [Header("Depth")]
        public Material _rgbDepthCameraShader;
        public float _depthScale = 100.0f;



        [Header("Internal")]
        public int _maxAsyncGPURequestQueue = 2;
        public int _maxPublishTaskQueue = 2;


        // ~~~~~~ Delegate Callbacks ~~~~~~
        /// <summary>
        /// Called every frame passing in:
        /// this, string cameraId, width, height, RGB24[] image, float32[] depth
        /// 
        /// Note: is async so returns a task
        /// </summary>
        /// 
        /// <value></value>
        public Func<ZORGBDepthCamera, string, int, int, byte[], float[], Task> OnPublishDelegate { get; set; }

        // ~~~~~~ Render Buffers ~~~~~~~
        private RenderTexture _colorBuffer;
        private RenderTexture _depthBuffer;

        // the color texture used for debug rendering
        private Texture2D _colorTexture;

        // the depth texture used for debug rendering
        private Texture2D _depthTexture;

        // the final color and depth render texture which is of type ARGBFloat 
        // Alpha channel contains depth image.
        private RenderTexture _colorDepthRenderTexture;
        private bool _isOpenGLRenderer = false;

        // ~~~~ Async GPU Read ~~~~ //
        Queue<AsyncGPUReadbackPluginRequest> _asyncGPURequests = new Queue<AsyncGPUReadbackPluginRequest>();

        // the final native arrays that contain the F32 depth values and RGB8 color values
        private NativeArray<float> _depthBufferValuesFloat;
        private NativeArray<byte> _rgbValues;

        // publish queue
        private Queue<Task> _publishTasks = new Queue<Task>();


        protected override void ZOReset() {
            base.ZOReset();
            UpdateRateHz = 30;
        }

        // Start is called before the first frame update
        protected override void ZOStart() {
            if (_camera == null) {
                _camera = this.GetComponent<Camera>();
            }

            // setup the buffers
            _colorBuffer = new RenderTexture(_width, _height, 0, RenderTextureFormat.ARGB32);
            _depthBuffer = new RenderTexture(_width, _height, 32, RenderTextureFormat.Depth);
            _camera.SetTargetBuffers(_colorBuffer.colorBuffer, _depthBuffer.depthBuffer);

            // setup depth texture mode for the camera
            _camera.depthTextureMode = _camera.depthTextureMode | DepthTextureMode.Depth;

            _colorTexture = new Texture2D(_width, _height, TextureFormat.RGB24, false);
            _depthTexture = new Texture2D(_width, _height, TextureFormat.RGBAFloat, false);

            // create the render texture.  RGB is color and alpha is depth.  Float32
            _colorDepthRenderTexture = new RenderTexture(_width, _height, 0, RenderTextureFormat.ARGBFloat);

            // create the final publish buffers where the color buffer is converted from Float32 to RGB8
            _depthBufferValuesFloat = new NativeArray<float>(_width * _height, Allocator.Persistent);
            _rgbValues = new NativeArray<byte>(_width * _height * 3, Allocator.Persistent);

            _depthBufferFloat = new float[_width * _height];
            _colorPixels24 = new byte[_width * _height * 3];

            if (SystemInfo.supportsAsyncGPUReadback == true) {
                Debug.Log("INFO: Native support for AsyncGPUReadback");
            } else {
                Debug.Log("WARNING: NO support for native AsyncGPUReadback. Using 3rd party.");
            }

            if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.OpenGLCore) {
                _isOpenGLRenderer = true;
            }

        }

        protected override void ZOFixedUpdateHzSynchronized() {
            _camera.Render();
        }

        protected override void ZOUpdate() {
            // clean up the publishing tasks
            while (_publishTasks.Count > 0 && _publishTasks.Peek().IsCompleted) {
                _publishTasks.Dequeue();
            }
            DoRenderTextureUpdate();
        }

        protected override void ZOOnGUI() {
            base.ZOOnGUI();
            GUI.DrawTexture(new Rect(10, 10, 320, 240), _colorBuffer, ScaleMode.ScaleToFit);
            GUI.DrawTexture(new Rect(10, 242, 320, 240), _depthBuffer, ScaleMode.ScaleToFit);
            GUI.DrawTexture(new Rect(325, 242, 320, 240), _colorDepthRenderTexture, ScaleMode.ScaleToFit, true);
        }


        private float[] _depthBufferFloat;
        private byte[] _colorPixels24;

        private void OnPostRender() {
            UnityEngine.Profiling.Profiler.BeginSample("ZORGBDepthCamera::OnPostRender");
            Rect cameraRect = new Rect(0, 0, _width, _height);
            _rgbDepthCameraShader.SetTexture("_MainTex", _colorBuffer);
            Graphics.Blit(_camera.targetTexture, _colorDepthRenderTexture, _rgbDepthCameraShader);

            if (_asyncGPURequests.Count < _maxAsyncGPURequestQueue) {
                _asyncGPURequests.Enqueue(AsyncGPUReadbackPlugin.Request(_colorDepthRenderTexture));
            }
            UnityEngine.Profiling.Profiler.EndSample();
        }

        private void DoRenderTextureUpdate() {
            // ~~~ Handle Async GPU Readback ~~~ //
            while (_asyncGPURequests.Count > 0) {
                var asyncGPURequest = _asyncGPURequests.Peek();
                // You need to explicitly ask for an update regularly
                asyncGPURequest.Update();

                if (asyncGPURequest.hasError) {
                    Debug.LogError("ERROR: GPU readback error detected.");
                    asyncGPURequest.Dispose();
                    _asyncGPURequests.Dequeue();
                } else if (asyncGPURequest.done) {
                    // Get data from the request when it's done
                    float[] rawTextureData = asyncGPURequest.GetRawData_ArrayFloat();
                    if (OnPublishDelegate != null) {
                        if (_publishTasks.Count < _maxPublishTaskQueue) {

                            Task publishTask = Task.Run(() => {
                                float inverseScale = 1.0f / _depthScale;
                                float r, g, b, d;
                                for (int z = 0, c = 0, p = 0; z < (_width * _height); z++, c += 3, p += 4) {
                                    r = rawTextureData[p];
                                    g = rawTextureData[p + 1];
                                    b = rawTextureData[p + 2];
                                    d = rawTextureData[p + 3];

                                    _colorPixels24[c + 0] = (byte)(b * 255.0f);
                                    _colorPixels24[c + 1] = (byte)(g * 255.0f);
                                    _colorPixels24[c + 2] = (byte)(r * 255.0f);
                                    _depthBufferFloat[z] = d * inverseScale;

                                }
                                OnPublishDelegate(this, _cameraId, _width, _height, _colorPixels24, _depthBufferFloat);
                            });

                            _publishTasks.Enqueue(publishTask);

                        } else {
                            Debug.Log("INFO: ZORGBDepthCamera publish task overflow...");
                        }

                    }

                    // You need to explicitly Dispose data after using them
                    asyncGPURequest.Dispose();

                    _asyncGPURequests.Dequeue();
                } else {
                    break;
                }
            }

        }
        private void OnDestroy() {
            _depthBufferValuesFloat.Dispose();
            _rgbValues.Dispose();
        }



    }

}
