using UnityEngine;
using System.Collections;
using System;
using System.Threading;
using System.Runtime.InteropServices;
using UnityEngine.Rendering;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace ZO.Util.Rendering {

    // Tries to match the official API
    public class ZOAsyncGPUReadbackPlugin {
        public static ZOAsyncGPUReadbackPluginRequest Request(Texture src) {
            return new ZOAsyncGPUReadbackPluginRequest(src);
        }
    }

    public class ZOAsyncGPUReadbackPluginRequest {

        /// <summary>
        /// Tell if we are using the plugin api or the official api
        /// </summary>
        private bool _usePlugin;

        /// <summary>
        /// Official api request object used if supported
        /// </summary>
        private AsyncGPUReadbackRequest _gpuRequest;

        /// <summary>
        /// Event Id used to tell what texture is targeted to the render thread
        /// </summary>
        private int _eventId;

        /// <summary>
        /// Is buffer allocated
        /// </summary>
        private bool _bufferCreated = false;

        /// <summary>
        /// Check if the request is done
        /// </summary>
        public bool done {
            get {
                if (_usePlugin) {
                    return isRequestDone(_eventId);
                } else {
                    return _gpuRequest.done;
                }
            }
        }

        /// <summary>
        /// Check if the request has an error
        /// </summary>
        public bool hasError {
            get {
                if (_usePlugin) {
                    return isRequestError(_eventId);
                } else {
                    return _gpuRequest.hasError;
                }
            }
        }

        /// <summary>
        /// Create an AsyncGPUReadbackPluginRequest.
        /// Use official AsyncGPUReadback.Request if possible.
        /// If not, it tries to use OpenGL specific implementation
        /// Warning! Can only be called from render thread yet (not main thread)
        /// </summary>
        /// <param name="src"></param>
        /// <returns></returns>
        public ZOAsyncGPUReadbackPluginRequest(Texture src) {
            if (SystemInfo.supportsAsyncGPUReadback) {
                _usePlugin = false;
                _gpuRequest = AsyncGPUReadback.Request(src, 0, TextureFormat.ARGB32);
            } else if (isCompatible()) {
                _usePlugin = true;
                int textureId = (int)(src.GetNativeTexturePtr());
                this._eventId = makeRequest_mainThread(textureId, 0);
                GL.IssuePluginEvent(getfunction_makeRequest_renderThread(), this._eventId);
            } else {
                Debug.LogError("AsyncGPUReadback is not supported on your system.");
            }
        }
        byte[] _buffer = null;
        public unsafe byte[] GetRawData() {
            if (_usePlugin) {
                // Get data from cpp plugin
                void* ptr = null;
                int length = 0;
                getData_mainThread(this._eventId, ref ptr, ref length);

                // Copy data to a buffer that we own and that will not be deleted
                if (_buffer == null || _buffer.Length != length) {
                    _buffer = new byte[length];
                }
                UnityEngine.Profiling.Profiler.BeginSample("Marshal.Copy");
                Marshal.Copy(new IntPtr(ptr), _buffer, 0, length);
                UnityEngine.Profiling.Profiler.EndSample();
                _bufferCreated = true;

                return _buffer;
            } else {
                return _gpuRequest.GetData<byte>().ToArray();
            }
        }

        /// <summary>
        /// Has to be called regularly to update request status.
        /// Call this from Update() or from a corountine
        /// </summary>
        /// <param name="force">Update is automatic on official api,
        /// so we don't call the Update() method except on force mode.</param>
        public void Update(bool force = false) {
            if (_usePlugin) {
                IntPtr ip = getfunction_update_renderThread();

                UnityEngine.Profiling.Profiler.BeginSample("IssuePluginEvent");
                GL.IssuePluginEvent(ip, this._eventId);
                UnityEngine.Profiling.Profiler.EndSample();
            } else if (force) {
                _gpuRequest.Update();
            }
        }

        /// <summary>
        /// Has to be called to free the allocated buffer after it has been used
        /// </summary>
        public void Dispose() {
            if (_usePlugin && _bufferCreated) {
                dispose(this._eventId);
            }
        }


        [DllImport("AsyncGPUReadbackPlugin")]
        private static extern bool isCompatible();
        [DllImport("AsyncGPUReadbackPlugin")]
        private static extern int makeRequest_mainThread(int texture, int miplevel);
        [DllImport("AsyncGPUReadbackPlugin")]
        private static extern IntPtr getfunction_makeRequest_renderThread();
        [DllImport("AsyncGPUReadbackPlugin")]
        private static extern void makeRequest_renderThread(int event_id);
        [DllImport("AsyncGPUReadbackPlugin")]
        private static extern IntPtr getfunction_update_renderThread();
        [DllImport("AsyncGPUReadbackPlugin")]
        private static extern unsafe void getData_mainThread(int event_id, ref void* buffer, ref int length);
        [DllImport("AsyncGPUReadbackPlugin")]
        private static extern bool isRequestError(int event_id);
        [DllImport("AsyncGPUReadbackPlugin")]
        private static extern bool isRequestDone(int event_id);
        [DllImport("AsyncGPUReadbackPlugin")]
        private static extern void dispose(int event_id);
    }
}