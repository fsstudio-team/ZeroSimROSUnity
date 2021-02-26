using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;

using ZO.Document;
using ZO.Util;
using ZO.Util.Extensions;


namespace ZO.Sensors {

    [RequireComponent(typeof(Rigidbody))]
    public class ZOIMU : ZOGameObjectBase, ZOSerializationInterface {



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
        private Vector3 _gravity = Vector3.zero;

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
        /// Vector3 linear_accel no gravity, 
        /// Vector3 gravity_accel,
        /// Vector3 angular_velocity,
        /// Quaternion orientation
        /// 
        /// Note: is async so returns a task
        /// </summary>
        /// 
        /// <value></value>
        public Func<ZOIMU, string, Vector3, Vector3, Vector3, Quaternion, Task> OnPublishDelegate { get; set; }


        private Rigidbody _rigidBody;
        private Vector3 _lastVelocity = Vector3.zero;

        public Vector3 Acceleration {
            get { return _acceleration; }
        }

        public Vector3 Gravity {
            get { return _gravity; }
        }


        public Vector3 AngularVelocity {
            get { return _angularVelocity; }
        }


        public Quaternion Orientation {
            get { return _orientation; }
        }


        #region ZOGameObjectBase

        // Start is called before the first frame update
        protected override void ZOStart() {
            _rigidBody = GetComponent<Rigidbody>();
        }

        protected override async void ZOFixedUpdateHzSynchronized() {

            // calculate linear acceleration
            Vector3 velocity = transform.InverseTransformDirection(_rigidBody.velocity);
            Vector3 prevAcceleration = _acceleration;
            _acceleration = (velocity - _lastVelocity) / Time.deltaTime;
            _lastVelocity = velocity;


            // calculate gravity
            _gravity = transform.InverseTransformDirection(UnityEngine.Physics.gravity);  
                      
            _acceleration = _linearNoise.Apply(_acceleration);
            _acceleration = ZO.Math.ZOMathUtil.lowPassFilter(_acceleration, prevAcceleration, _preFilterAccelerationAlpha);

            _angularVelocity = _rigidBody.angularVelocity;
            _angularVelocity = _angularNoise.Apply(_angularVelocity);

            _orientation = transform.rotation;

            Vector3 publishedGravity = _gravity;
            Vector3 publishedAcceleration = _acceleration;
            Vector3 publishedAngularVelocity = _angularVelocity;
            Quaternion publishedOrientation = transform.rotation;


            if (OnPublishDelegate != null) {
                await OnPublishDelegate(this, Name, publishedAcceleration, publishedGravity, publishedAngularVelocity, publishedOrientation);
            }

        }

        #endregion // ZOGameObjectBase

        /* # Notes:

            * To calculate angle using complementary filter
                * http://www.pieter-jan.com/node/11
            * C# code https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU/blob/master/x-IMU%20IMU%20and%20AHRS%20Algorithms/x-IMU%20IMU%20and%20AHRS%20Algorithms/AHRS/MadgwickAHRS.cs  line 212 

        */

        #region ZOSerializationInterface
        public string Type {
            get { return "sensor.imu"; }
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
                new JProperty("update_rate_hz", UpdateRateHz)
            );


            ZOSimOccurrence parent_occurrence = GetComponent<ZOSimOccurrence>();
            if (parent_occurrence) {
                json["parent_occurrence"] = parent_occurrence.Name;
            }

            _json = json;

            return json;
        }


        public void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            // Assert.Equals(json["type"].Value<string>() == Type);
            _json = json;
            Name = json.ValueOrDefault("name", Name);
            UpdateRateHz = json.ValueOrDefault("update_rate_hz", UpdateRateHz);


        }
        #endregion // ZOSerializationInterface


    }

}
