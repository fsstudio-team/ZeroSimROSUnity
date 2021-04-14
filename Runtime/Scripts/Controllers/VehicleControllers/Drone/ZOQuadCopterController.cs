using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ZO.Util;
using ZO.Physics;
using ZO.Sensors;

namespace ZO.Controllers {

    public class ZOQuadCopterController : ZOGameObjectBase {

        public enum ZOQuadCopterMotorConfiguration {
            XConfiguration,
            CrossConfiguration
        }

        public ZOQuadCopterMotorConfiguration _quadCopterConfiguration = ZOQuadCopterMotorConfiguration.XConfiguration;
        public ZOQuadCopterMotorConfiguration QuadCopterConfiguration {
            get => _quadCopterConfiguration;
        }
        public float _rotorDistanceFromCenter = 0.2f;
        public float RotorDistanceFromCenter {
            get => _rotorDistanceFromCenter;
        }
        public float _maxPerRotorForce = 50.0f;

        public Rigidbody _baseRigidBody = null;
        public Rigidbody BaseRigidBody {
            get => _baseRigidBody;
        }

        [Header("Altitude Control")]
        public ZOAltimeter _altimeter;
        public ZOAltimeter Altimeter {
            get => _altimeter;
        }
        public ZOPIDController _altitudePID = new ZOPIDController {
            Kp = 10,
            Ki = 1,
            Kd = 1,
            MaximumOutputValue = 100.0f,
            DeadBandEpsilon = 0.01f
        };
        public ZOPIDController AltitudePID {
            get => _altitudePID;
        }

        private float AltitudeSetPoint {
            get {
                return AltitudePID.SetPoint;
            }
            set {
                AltitudePID.SetPoint = value;
            }
        }

        private float _currentThrottle = 0.0f;
        private float _throttleIncrement = 0.1f;

        protected class Motor {
            public Vector3 localPosition = new Vector3(0,0,0);
            public Vector3 globalPosition = new Vector3(0,0,0);

            public float force = 0;
            public Vector3 globalForceVector = new Vector3(0,0,0);
        }

        protected Motor[] _motors = new Motor[4];
        protected Motor[] Motors {
            get => _motors;
        }

        protected override void ZOStart() {
            base.ZOStart();
            
            // initialize the motors
            for (int i = 0; i < Motors.Length; i++) {
                Motors[i] = new Motor();
            }

            // set the altitude setpoint
            if (Altimeter != null) {
                AltitudeSetPoint = Altimeter.AltitudeMeters;
            }
        }


        protected override void ZOUpdate() {
            base.ZOUpdate();

            if (Input.GetKeyDown("i")) {
                AltitudeSetPoint += _throttleIncrement;
            }
            if (Input.GetKeyDown("k")) {
                AltitudeSetPoint -= _throttleIncrement;
            }

        }

        protected override void ZOFixedUpdate() {
            base.ZOFixedUpdate();
            if (QuadCopterConfiguration == ZOQuadCopterMotorConfiguration.XConfiguration) {

                // calculate altitude control
                _currentThrottle = AltitudePID.Update(Altimeter.AltitudeMeters, Time.deltaTime);
                // forward right motor
                Vector3 forwardRightPosition = BaseRigidBody.transform.TransformPoint(new Vector3(RotorDistanceFromCenter, 0, RotorDistanceFromCenter));                
                Vector3 forwardRightForce = BaseRigidBody.transform.up * _currentThrottle;
                BaseRigidBody.AddForceAtPosition(forwardRightForce, forwardRightPosition);
                Motors[0].globalPosition = forwardRightPosition;
                Motors[0].globalForceVector = forwardRightForce;
                
                // forward left motor
                Vector3 forwardLeftPosition = BaseRigidBody.transform.TransformPoint(new Vector3(-RotorDistanceFromCenter, 0, RotorDistanceFromCenter));
                Vector3 forwardLeftForce = BaseRigidBody.transform.up * _currentThrottle;
                BaseRigidBody.AddForceAtPosition(forwardLeftForce, forwardLeftPosition);
                Motors[1].globalPosition = forwardLeftPosition;
                Motors[1].globalForceVector = forwardLeftForce;

                // back right motor
                Vector3 backRightPosition = BaseRigidBody.transform.TransformPoint(new Vector3(RotorDistanceFromCenter, 0, -RotorDistanceFromCenter));
                Vector3 backRightForce = BaseRigidBody.transform.up * _currentThrottle;
                BaseRigidBody.AddForceAtPosition(backRightForce, backRightPosition);
                Motors[2].globalPosition = backRightPosition;
                Motors[2].globalForceVector = backRightForce;

                // back left motor
                Vector3 backLeftPosition = BaseRigidBody.transform.TransformPoint(new Vector3(-RotorDistanceFromCenter, 0, -RotorDistanceFromCenter));
                Vector3 backLeftForce = BaseRigidBody.transform.up * _currentThrottle;
                BaseRigidBody.AddForceAtPosition(backLeftForce, backLeftPosition);
                Motors[3].globalPosition = backLeftPosition;
                Motors[3].globalForceVector = backLeftForce;

            } else if (QuadCopterConfiguration == ZOQuadCopterMotorConfiguration.CrossConfiguration) {
                //TODO:
            }

        }

        

        private void OnGUI() {
            GUI.TextField(new Rect(10, 10, 300, 22), $"Altitude Set Point: {AltitudeSetPoint.ToString("R2")}");
            GUI.TextField(new Rect(10, 25, 300, 22), $"Altitude: {Altimeter.AltitudeMeters.ToString("R2")}");
            GUI.TextField(new Rect(10, 40, 300, 22), $"Altitude Throttle: {_currentThrottle.ToString("R2")}");

            foreach (Motor motor in Motors) {
                Debug.DrawRay(motor.globalPosition, motor.globalForceVector, Color.green, 0.1f);
            }

        }
    }
}