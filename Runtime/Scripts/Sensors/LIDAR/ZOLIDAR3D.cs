using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using ZO.Util;

namespace ZO.Sensors {

    // this job creates a batch of RaycastCommands we are going to use for
    // collision detection against the world. these can be sent to PhysX
    // as a batch that will be executed in a job, rather than us having to
    // call Physics.Raycast in a loop just on the main thread! 
    [BurstCompile(CompileSynchronously = true)]
    struct ZOPrepareRaycastCommands : IJobParallelFor {
        public Vector3 Position;
        public float Distance;
        public NativeArray<RaycastCommand> RaycastCommands;
        [ReadOnly]
        public NativeArray<Vector3> RaycastDirections;

        public void Execute(int i) {
            RaycastCommands[i] = new RaycastCommand(Position, RaycastDirections[i], Distance);
        }
    };

    [BurstCompile(CompileSynchronously = true)]
    struct ZOTransformHits : IJobParallelFor {
        [ReadOnly] public ZOLIDAR3D.ReferenceFrame ReferenceFrame;
        [ReadOnly] public Vector3 Position;

        [ReadOnly] public NativeArray<RaycastHit> RaycastHits;

        public NativeArray<Vector3> HitPositions;
        public NativeArray<Vector3> HitNormals;


        public void Execute(int i) {

            RaycastHit hit = RaycastHits[i];
            if (ReferenceFrame == ZOLIDAR3D.ReferenceFrame.LeftHanded_XRight_YUp_ZForward) {
                HitNormals[i] = hit.normal;
                HitPositions[i] = hit.point - Position;
            } else if (ReferenceFrame == ZOLIDAR3D.ReferenceFrame.RightHanded_XBackward_YLeft_ZUp) {
                HitNormals[i] = new Vector3(-hit.normal.z, -hit.normal.x, hit.normal.y);
                Vector3 pos = hit.point - Position;
                HitPositions[i] = new Vector3(-pos.z, -pos.x, pos.y);
            }
        }
    };

    public class ZOLIDAR3D : ZOGameObjectBase {
        /// <summary>
        /// A generic LIDAR sensor.  
        /// </summary>

        public string _lidarId = "Not Set";
        public enum ReferenceFrame {
            RightHanded_XBackward_YLeft_ZUp,
            LeftHanded_XRight_YUp_ZForward // Unity Standard
        };

        public ReferenceFrame _referenceFrame = ReferenceFrame.LeftHanded_XRight_YUp_ZForward;



        [Header("FOV")]
        public float _verticalUpFovDegrees = 21.0f;
        public float _verticalDownFovDegrees = 74.0f;
        public float _horizontalFovDegrees = 360.0f;

        [Header("Resolution")]
        public float _verticalResolutionDegrees = 0.76f;
        public float _horizontalResolutionDegrees = 1.2f;
        public float _minRange = 0.0f;
        public float _maxRange = 20.0f;

        // ~~~~~~ Delegate Callbacks ~~~~~~
        /// <summary>
        /// Called every frame passing in:
        /// ZOLIDAR, string lidar id, int number of hits, Vector3 position, Vector3 normals
        /// 
        /// Note: is async so returns a task
        /// </summary>
        /// 
        /// <value></value>
        public Func<ZOLIDAR3D, string, NativeArray<Vector3>, NativeArray<Vector3>, Task> OnPublishDelegate { get; set; }

        // Property Accessors
        public float HorizontalFOVDegrees { get => _horizontalFovDegrees; }
        public float HorizontalResolutionDegrees { get => _horizontalResolutionDegrees; }
        public float MinRange { get => _minRange; }
        public float MaxRange { get => _maxRange; }


        private int _horizontalRayCount = -1;
        private int _verticalScanCount = -1;
        private int _totalRayCount = -1;

        private ZORaycastJobBatch _raycastBatchJob;
        private RaycastHit[] _rayCastHits;

        private NativeArray<Vector3> _hitPositions;
        private NativeArray<Vector3> _hitNormals;

        /// Pre-calculated rays
        NativeArray<Vector3> _rayDirections;

        ///
        JobHandle _transformHitsJobHandle = default(JobHandle);

        // Start is called before the first frame update
        protected override void ZOStart() {

            Debug.Log("INFO: ZOLIDAR::Start");

            _horizontalRayCount = Mathf.RoundToInt(_horizontalFovDegrees / _horizontalResolutionDegrees);
            _verticalScanCount = Mathf.RoundToInt((_verticalDownFovDegrees + _verticalUpFovDegrees) / _verticalResolutionDegrees);
            _totalRayCount = _horizontalRayCount * _verticalScanCount;

            _raycastBatchJob = new ZORaycastJobBatch(_totalRayCount, Allocator.TempJob);


            _rayCastHits = new RaycastHit[_totalRayCount];

            // build up the ray directions
            _rayDirections = new NativeArray<Vector3>(_totalRayCount, Allocator.Persistent);
            Vector3 rayDirection = Quaternion.AngleAxis(_verticalUpFovDegrees, transform.right) * transform.forward;
            Quaternion horizontalRotationStep = Quaternion.AngleAxis(_horizontalResolutionDegrees, transform.up);
            Quaternion verticalRotationStep = Quaternion.AngleAxis(-_verticalResolutionDegrees, transform.right);
            int rayIndex = 0;
            for (int verticalStep = 0; verticalStep < _verticalScanCount; verticalStep++) {
                for (int horizontalStep = 0; horizontalStep < _horizontalRayCount; horizontalStep++) {
                    _rayDirections[rayIndex] = rayDirection;
                    rayIndex++;
                    rayDirection = horizontalRotationStep * rayDirection;
                }

                // BUGBUG:??? transform.right may change so do we need to have a rightDirection that moves with the rayDirection
                rayDirection = verticalRotationStep * rayDirection;
            }

            _hitPositions = new NativeArray<Vector3>(_totalRayCount, Allocator.Persistent);
            _hitNormals = new NativeArray<Vector3>(_totalRayCount, Allocator.Persistent);

        }


        private void OnDestroy() {
            _transformHitsJobHandle.Complete();
            _rayDirections.Dispose();
            _hitNormals.Dispose();
            _hitPositions.Dispose();
            _raycastBatchJob.Dispose();
        }

        protected override async void ZOFixedUpdateHzSynchronized() {
            UnityEngine.Profiling.Profiler.BeginSample("ZOLIDAR::ZOUpdateHzSynchronized");
            if (_transformHitsJobHandle.IsCompleted == true) {

                _transformHitsJobHandle.Complete();

                if (OnPublishDelegate != null) {
                    UnityEngine.Profiling.Profiler.BeginSample("ZOLIDAR::ZOUpdateHzSynchronized::Publish");
                    await OnPublishDelegate(this, _lidarId, _hitPositions, _hitNormals);
                    UnityEngine.Profiling.Profiler.EndSample();
                }

                // Ref: https://github.com/LotteMakesStuff/SimplePhysicsDemo/blob/master/Assets/SimpleJobifiedPhysics.cs
                // create new raycast job
                _raycastBatchJob.Dispose();
                _raycastBatchJob = new ZORaycastJobBatch(_totalRayCount, Allocator.TempJob);
                // SetupRaycasts();
                var setupRaycastsJob = new ZOPrepareRaycastCommands() {
                    Position = transform.position,
                    Distance = _maxRange,
                    RaycastDirections = _rayDirections,
                    RaycastCommands = _raycastBatchJob.RaycastCommands
                };
                JobHandle setupRaycastsJobHandle = setupRaycastsJob.Schedule(_totalRayCount, 32);

                _raycastBatchJob.Schedule(32, setupRaycastsJobHandle);

                var transformHitJob = new ZOTransformHits() {
                    ReferenceFrame = _referenceFrame,
                    Position = transform.position,
                    RaycastHits = _raycastBatchJob.RaycastHitResults,
                    HitPositions = _hitPositions,
                    HitNormals = _hitNormals
                };

                _transformHitsJobHandle = transformHitJob.Schedule(_totalRayCount, 32, _raycastBatchJob.RaycastBatchJobHandle);


            }
            UnityEngine.Profiling.Profiler.EndSample();

        }

    }
}