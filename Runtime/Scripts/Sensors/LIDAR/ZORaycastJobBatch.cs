using UnityEngine;
using Unity.Collections;
using Unity.Jobs;

namespace ZO.Sensors {
    public struct ZORaycastJobBatch : System.IDisposable {

        private NativeArray<RaycastCommand> _raycastCommands;
        public NativeArray<RaycastCommand> RaycastCommands {
            get { return _raycastCommands; }
        }
        public void SetCommand(int index, Vector3 start, Vector3 direction, float maxDistance) {
            UnityEngine.Profiling.Profiler.BeginSample("ZORaycastJobBatch::SetCommand");
            _raycastCommands[index] = new RaycastCommand(start, direction, maxDistance);
            UnityEngine.Profiling.Profiler.EndSample();
        }
        private NativeArray<RaycastHit> _raycastHitresults;
        public NativeArray<RaycastHit> RaycastHitResults {
            get { return _raycastHitresults; }
        }
        private JobHandle _raycastBatchJobHandle;
        public JobHandle RaycastBatchJobHandle => _raycastBatchJobHandle;

        Allocator _allocator;

        public bool IsCompleted {
            get { return _raycastBatchJobHandle.IsCompleted; }
        }

        public int Length {
            get {
                int numCommands = _raycastCommands.Length;
                int numResults = _raycastHitresults.Length;
                return numCommands == numResults ? numCommands : -1;//-1 when array lengths are not equal
            }
            set {
                if (Length != value) {
                    Dispose();
                    _raycastCommands = new NativeArray<RaycastCommand>(value, _allocator);
                    _raycastHitresults = new NativeArray<RaycastHit>(value, _allocator, NativeArrayOptions.UninitializedMemory);
                }
            }
        }



        public ZORaycastJobBatch(int length, Allocator allocator) {
            _allocator = allocator;
            _raycastCommands = new NativeArray<RaycastCommand>(length, _allocator);
            _raycastHitresults = new NativeArray<RaycastHit>(length, _allocator, NativeArrayOptions.UninitializedMemory);
            _raycastBatchJobHandle = default(JobHandle);
        }



        public void Schedule(int minCommandsPerJob = 32, JobHandle dependsOn = default(JobHandle)) {
#if DEBUG
            if (_raycastBatchJobHandle.IsCompleted == false) {
                Debug.LogWarning("WARNING: Scheduling when job is not complete");
            }
#endif
            _raycastBatchJobHandle = RaycastCommand.ScheduleBatch(_raycastCommands, _raycastHitresults, minCommandsPerJob, dependsOn);
        }


        public void Complete() {
            _raycastBatchJobHandle.Complete();
            _raycastBatchJobHandle = default(JobHandle);
        }

        public void CopyResults(ref RaycastHit[] array) {
#if DEBUG
            if (_raycastBatchJobHandle.IsCompleted == false) {
                Debug.LogWarning("WARNING: Copying results when job is not complete");
            }
#endif
            if (array.Length != _raycastHitresults.Length) {
                System.Array.Resize(ref array, _raycastHitresults.Length);
            }
            _raycastHitresults.CopyTo(array);
        }

        public void Dispose() {
            Complete();
            if (_raycastCommands.IsCreated) {
                _raycastCommands.Dispose();
            }
            if (_raycastHitresults.IsCreated) {
                _raycastHitresults.Dispose();
            }
        }

    }

}