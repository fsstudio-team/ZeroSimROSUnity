using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using ZO.Util;

namespace ZO.Sensors {

    /// <summary>
    /// ranges[i] = laserScan.ranges[i];
    /// directions[i] = new Vector3(Mathf.Cos(laserScan.angle_min + laserScan.angle_increment * i), Mathf.Sin(laserScan.angle_min + laserScan.angle_increment * i), 0).Ros2Unity();
 
    /// </summary>

    public class ZOLIDAR2D : ZOGameObjectBase {
        /// <summary>
        /// A generic 2D LIDAR sensor.  
        /// </summary>

        public string _lidarId = "Not Set";



        [Header("FOV")]
        public float _angleMinDegrees = 0.0f;
        public float _angleMaxDegrees = 360.0f;
        public float _angleIncrementDegrees = 1.0f;

        public float _rangeMin = 0.1f;
        public float _rangeMax = 20.0f;

        // ~~~~~~ Delegate Callbacks ~~~~~~
        /// <summary>
        /// Called every frame passing in:
        /// ZOLIDAR, string lidar id, int number of hits, Vector3 position, Vector3 normals
        /// 
        /// Note: is async so returns a task
        /// </summary>
        /// 
        /// <value></value>
        public Func<ZOLIDAR2D, string, float[], Task> OnPublishDelegate { get; set; }

        // Property Accessors
        public float AngelMinDegrees { get => _angleMinDegrees; }
        public float AngleMaxDegrees { get => _angleMaxDegrees; }
        public float RangeMin { get => _rangeMin; }
        public float RangeMax { get => _rangeMax; }
        public float AngleIncrementDegrees { get => _angleIncrementDegrees; }
        public float FOVDegrees { 
            get {
                return AngleMaxDegrees - AngelMinDegrees;
            }
        }

        public string LidarID {
            get => _lidarId;
            set => _lidarId = value;
        }



        /// <summary>
        /// Time between scans in seconds.
        /// </summary>
        /// <value>seconds</value>
        public float ScanTimeSeconds {
            get { 
                return UpdateRateHz > 0.0f ? 1.0f/UpdateRateHz : 1.0f/100.0f;
            }
        }


        /// <summary>
        ///  time between measurements [seconds] - if your scanner
        /// is moving, this will be used in interpolating position
        /// of 3d points
        /// </summary>
        /// <value>seconds</value>
        public float TimeIncrementSeconds {
            get {
                return ScanTimeSeconds / (float)RayCount;
            }
        }

        public int RayCount {
            get => _rayCount;
        }


        private int _rayCount = -1;
        private Ray[] _rays;
        private RaycastHit[] _raycastHits;
        private float[] _ranges;
        // Start is called before the first frame update
        protected override void ZOStart() {
            Debug.Log("INFO: ZOLIDAR2D::Start");
            _rayCount = Mathf.RoundToInt(FOVDegrees / AngleIncrementDegrees);
            _rays = new Ray[_rayCount];
            _raycastHits = new RaycastHit[_rayCount];
            _ranges = new float[_rayCount];            
        }


        private void OnDestroy() {
        }

        protected override void ZOFixedUpdateHzSynchronized() {
            UnityEngine.Profiling.Profiler.BeginSample("ZOLIDAR2D::ZOUpdateHzSynchronized");
            // do raycasts
            // TODO: use batch raycasts like the 3d raycast
            for (int i = 0; i < _rayCount; i++) {
                Vector3 axis = new Vector3(0, AngelMinDegrees - AngleIncrementDegrees * i, 0);
                Vector3 direction = Quaternion.Euler(axis) * transform.forward;
                _rays[i] = new Ray(transform.position, direction);
                _ranges[i] = 0;

                _raycastHits[i] = new RaycastHit();
                if (UnityEngine.Physics.Raycast(_rays[i], out _raycastHits[i], RangeMax)) {
                    if (_raycastHits[i].distance >= RangeMin && _raycastHits[i].distance <= RangeMax) {
                        _ranges[i] = _raycastHits[i].distance;

                        if (IsDebug == true) {
                            Debug.DrawLine(transform.position, _raycastHits[i].point, Color.green, ScanTimeSeconds);
                        }
                    } 
                } 
            }

            // publish
            if (OnPublishDelegate != null) {
                OnPublishDelegate(this, LidarID, _ranges);
            }

            UnityEngine.Profiling.Profiler.EndSample();

        }

    }
}