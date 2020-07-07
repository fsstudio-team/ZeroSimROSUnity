using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

namespace ZO.Util {

    [RequireComponent(typeof(Rigidbody))]
    public class ZOIMU : ZOGameObjectBase {

        [Header("IMU Parametrs")]
        public String _imuId = "none";

        // RightHanded_XBackward_YLeft_ZUp,
        // LeftHanded_XRight_YUp_ZForward // Unity Standard

        public enum ReferenceFrame {
            RightHanded_XBackward_YLeft_ZUp,
            LeftHanded_XRight_YUp_ZForward // Unity Standard
        };

        public ReferenceFrame _referenceFrame = ReferenceFrame.LeftHanded_XRight_YUp_ZForward;

        [Header("Noise Models")]
        /// <summary>
        /// Angular Noise in Radians per Second
        /// </summary>
        public ZO.Math.ZOGaussianNoiseModel _angularNoise = new ZO.Math.ZOGaussianNoiseModel {
            Mean = 0.0,
            StdDev = 2e-4,
            BiasMean = 75e-6,
            BiasStdDev = 8e-08
        };

        /// <summary>
        /// Linear Noise in Meters per Seconds
        /// </summary>
        public ZO.Math.ZOGaussianNoiseModel _linearNoise = new ZO.Math.ZOGaussianNoiseModel {
            Mean = 0.0,
            StdDev = 1.7e-2f,
            BiasMean = 0.1,
            BiasStdDev = 1e-04
        };

        [Header("Filters")]
        public float _preFilterAccelerationAlpha = 0.5f;

        [Header("Read Only State")]
        [ZO.Util.ZOReadOnly]
        [SerializeField]
        private Vector3 _acceleration = Vector3.zero;
        [ZO.Util.ZOReadOnly]
        [SerializeField]
        private Vector3 _angularVelocity = Vector3.zero;
        [ZO.Util.ZOReadOnly]
        [SerializeField]
        private Quaternion _orientation = Quaternion.identity;

        // ~~~~~~ Delegate Callbacks ~~~~~~
        /// <summary>
        /// Called every frame passing in:
        /// this, 
        /// string imuID, 
        /// Vector3 linear_accel, 
        /// Vector3 angular_velocity,
        /// Quaternion orientation
        /// 
        /// Note: is async so returns a task
        /// </summary>
        /// 
        /// <value></value>
        public Func<ZOIMU, string, Vector3, Vector3, Quaternion, Task> OnPublishDelegate { get; set; }


        private Rigidbody _rigidBody;
        private Vector3 _lastVelocity = Vector3.zero;

        public Vector3 Acceleration {
            get { return _acceleration; }
        }


        public Vector3 AngularVelocity {
            get { return _angularVelocity; }
        }


        public Quaternion Orientation {
            get { return _orientation; }
        }

        // Start is called before the first frame update
        void Start() {
            _rigidBody = GetComponent<Rigidbody>();
        }

        protected override async void ZOFixedUpdate() {

            // calculate linear acceleration
            Vector3 velocity = transform.InverseTransformDirection(_rigidBody.velocity);
            Vector3 prevAcceleration = _acceleration;
            _acceleration = (velocity - _lastVelocity) / Time.fixedDeltaTime;
            _lastVelocity = velocity;
            _acceleration += transform.InverseTransformDirection(UnityEngine.Physics.gravity);
            _acceleration = _linearNoise.Apply(_acceleration);
            _acceleration = ZO.Math.ZOMathUtil.lowPassFilter(_acceleration, prevAcceleration, _preFilterAccelerationAlpha);

            _angularVelocity = _rigidBody.angularVelocity;
            _angularVelocity = _angularNoise.Apply(_angularVelocity);

            _orientation = transform.rotation;

            Vector3 publishedAcceleration = _acceleration;
            Vector3 publishedAngularVelocity = _angularVelocity;
            Quaternion publishedOrientation = transform.rotation;

            if (_referenceFrame == ReferenceFrame.RightHanded_XBackward_YLeft_ZUp) {
                //  (x, y, z, w) -> (-x, -z, y, -w).
                publishedOrientation = new Quaternion(-transform.rotation.z, -transform.rotation.x, transform.rotation.y, -transform.rotation.w);
                // if (m_zReverse)
                //     rot = new Quaternion (-rot.x, rot.y, -rot.z, rot.w);     
                publishedAcceleration = new Vector3(-_acceleration.z, -_acceleration.x, _acceleration.y);
                publishedAngularVelocity = new Vector3(-_angularVelocity.z, -_angularVelocity.x, _angularVelocity.y);
            }

            if (OnPublishDelegate != null) {
                await OnPublishDelegate(this, _imuId, publishedAcceleration, publishedAngularVelocity, publishedOrientation);
            }

        }

        /* # Notes:

            * To calculate angle using complementary filter
                * http://www.pieter-jan.com/node/11
            * C# code https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU/blob/master/x-IMU%20IMU%20and%20AHRS%20Algorithms/x-IMU%20IMU%20and%20AHRS%20Algorithms/AHRS/MadgwickAHRS.cs  line 212 

        */



    }

}
