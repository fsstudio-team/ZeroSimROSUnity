using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.Util;
using ZO.Util.Extensions;
using ZO.ROS.Unity;
using ZO.Document;

namespace ZO.Sensors {


    /// <summary>
    /// 2D LIDAR Simulation.
    /// </summary>
    public class ZOLIDAR2D : ZOGameObjectBase {


        [Header("FOV")]
        public float _minAngleDegrees = 0.0f;
        public float _maxAngleDegrees = 360.0f;
        public float _angleIncrementDegrees = 1.0f;

        public float _minRangeDistanceMeters = 0.1f;
        public float _maxRangeDistanceMeters = 20.0f;

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
        /// <summary>
        /// Minimum angle FOV.  For a full 360 set minimum angle to 0 and maximum angle to 360.
        /// </summary>
        /// <value></value>
        public float MinAngleDegrees {
            get => _minAngleDegrees;
            set => _minAngleDegrees = value;
        }

        /// <summary>
        /// Maximum angle FOV.  For a full 360 set minimum angle to 0 and maximum angle to 360.
        /// </summary>
        /// <value></value>
        public float MaxAngleDegrees {
            get => _maxAngleDegrees;
            set => _maxAngleDegrees = value;
        }

        /// <summary>
        /// Minimum distance range of the laser.
        /// </summary>
        /// <value></value>
        public float MinRangeDistanceMeters {
            get => _minRangeDistanceMeters;
            set => _minRangeDistanceMeters = value;
        }

        /// <summary>
        /// Maximum distance range of the laser
        /// </summary>
        /// <value></value>
        public float MaxRangeDistanceMeters {
            get => _maxRangeDistanceMeters;
            set => _maxRangeDistanceMeters = value;
        }

        /// <summary>
        /// The scan increment in degrees of the laser.
        /// </summary>
        /// <value></value>
        public float AngleIncrementDegrees {
            get => _angleIncrementDegrees;
            set => _angleIncrementDegrees = value;
        }

        /// <summary>
        /// Field of View in degrees.
        /// </summary>
        /// <value></value>
        public float FOVDegrees {
            get {
                return MaxAngleDegrees - MinAngleDegrees;
            }
        }



        /// <summary>
        /// Time between scans in seconds.
        /// </summary>
        /// <value>seconds</value>
        public float ScanTimeSeconds {
            get {
                return UpdateRateHz > 0.0f ? 1.0f / UpdateRateHz : 1.0f / 100.0f;
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

        #region ZOGameObjectBase        
        // Start is called before the first frame update
        protected override void ZOStart() {
            base.ZOStart();
            Debug.Log("INFO: ZOLIDAR2D::Start");
            _rayCount = Mathf.RoundToInt(FOVDegrees / AngleIncrementDegrees);
            _rays = new Ray[_rayCount];
            _raycastHits = new RaycastHit[_rayCount];
            _ranges = new float[_rayCount];
        }


        protected override void ZOOnValidate() {
            base.ZOOnValidate();
            if (UpdateRateHz == 0) {
                UpdateRateHz = 10;
            }
            if (Name == "") {
                Name = gameObject.name + "_" + Type;
            }
        }

        protected override void ZOFixedUpdateHzSynchronized() {
            UnityEngine.Profiling.Profiler.BeginSample("ZOLIDAR2D::ZOUpdateHzSynchronized");
            // do raycasts
            // TODO: use batch raycasts like the 3d raycast
            for (int i = 0; i < _rayCount; i++) {
                Vector3 axis = new Vector3(0, -1 * MinAngleDegrees - AngleIncrementDegrees * i, 0);
                Vector3 direction = Quaternion.Euler(axis) * transform.forward;
                _rays[i] = new Ray(transform.position, direction);
                _ranges[i] = 0;

                _raycastHits[i] = new RaycastHit();
                if (UnityEngine.Physics.Raycast(_rays[i], out _raycastHits[i], MaxRangeDistanceMeters)) {
                    if (_raycastHits[i].distance >= MinRangeDistanceMeters && _raycastHits[i].distance <= MaxRangeDistanceMeters) {
                        _ranges[i] = _raycastHits[i].distance;

                        if (IsDebug == true) {
                            Debug.DrawLine(transform.position, _raycastHits[i].point, Color.green, ScanTimeSeconds);
                        }
                    }
                }
            }

            // publish
            if (OnPublishDelegate != null) {
                OnPublishDelegate(this, Name, _ranges);
            }

            UnityEngine.Profiling.Profiler.EndSample();

        }
        #endregion // ZOGameObjectBase

        #region ZOSerializationInterface
        public string Type {
            get { return "sensor.lidar2d"; }
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

        #endregion


    }
}