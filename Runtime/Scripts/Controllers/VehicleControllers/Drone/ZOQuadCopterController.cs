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

        public ZOIMU _imu;
        public ZOIMU IMU {
            get => _imu;
        }

        [Header("Altitude Control")]
        public ZOAltimeter _altimeter;
        public ZOAltimeter Altimeter {
            get => _altimeter;
        }
        public ZOPIDController _altitudeHoldPID = new ZOPIDController {
            Kp = 50,
            Ki = 15,
            Kd = 3,
            MaximumOutputValue = 1000.0f,
            DeadBandEpsilon = 0.0f
        };

        /// <summary>
        /// PID for altitude hold control
        /// </summary>
        /// <value></value>
        public ZOPIDController AltitudeHoldPID {
            get => _altitudeHoldPID;
        }

        public ZOPIDController _altitudeClimbPID = new ZOPIDController {
            Kp = 50,
            Ki = 15,
            Kd = 3,
            MaximumOutputValue = 100.0f,
            DeadBandEpsilon = 0.0f
        };

        public ZOPIDController AltitudeClimbPID {
            get => _altitudeClimbPID;
        }

        public float _altitudeClimbVelocity = 1.0f;
        public float AltitudeClimbVelocity {
            get => _altitudeClimbVelocity;
        }

        public enum AltitudeControlStateEnum {
            AltitudeHold,
            AltitudeClimb
        }

        public AltitudeControlStateEnum AltitudeControlState {
            get; set;
        } = AltitudeControlStateEnum.AltitudeHold;


        [Header("Attitude Control")]
        public ZOPIDController _rollControlPID = new ZOPIDController {
            Kp = 1,
            Ki = 0,
            Kd = 0.1f,
            MaximumOutputValue = 100.0f,
            DeadBandEpsilon = 0.0f
        };

        public ZOPIDController RollControllPID {
            get => _rollControlPID;
        }


        public ZOPIDController _pitchControlPID = new ZOPIDController {
            Kp = 1,
            Ki = 0,
            Kd = 0.1f,
            MaximumOutputValue = 100.0f,
            DeadBandEpsilon = 0.0f
        };

        public ZOPIDController PitchControllPID {
            get => _pitchControlPID;
        }

        private float _pitchForce = 0.0f;
        private float _rollForce = 0.0f;


        private float _altitudeForce = 0.0f;
        private float _altitudeHeightIncrement = 0.1f;

        protected class Motor {
            public Vector3 localPosition = new Vector3(0, 0, 0);
            public Vector3 globalPosition = new Vector3(0, 0, 0);

            public float force = 0;
            public Vector3 globalForceVector = new Vector3(0, 0, 0);
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
                AltitudeHoldPID.SetPoint = Altimeter.AltitudeMeters;
            }

            PitchControllPID.SetPoint = 0;
            RollControllPID.SetPoint = 0;
        }


        protected override void ZOUpdate() {
            base.ZOUpdate();

            if (Input.GetKeyDown("i")) {
                AltitudeHoldPID.SetPoint += _altitudeHeightIncrement;
            }
            if (Input.GetKeyDown("k")) {
                AltitudeHoldPID.SetPoint -= _altitudeHeightIncrement;
            }
            if (Input.GetKeyDown("w")) {
                PitchControllPID.SetPoint = 10.0f;
            }
            if (Input.GetKeyDown("x")) {
                PitchControllPID.SetPoint = -10.0f;
            }
            if (Input.GetKeyDown("d")) {
                RollControllPID.SetPoint = -10.0f;
            }
            if (Input.GetKeyDown("a")) {
                RollControllPID.SetPoint = 10.0f;
            }

            if (Input.GetKeyDown("s")) {
                PitchControllPID.SetPoint = 0.0f;
                RollControllPID.SetPoint = 0.0f;
            }

        }

        protected override void ZOFixedUpdate() {
            base.ZOFixedUpdate();

            // zero out forces
            foreach (Motor motor in Motors) {
                motor.globalForceVector = Vector3.zero;
            }

            if (QuadCopterConfiguration == ZOQuadCopterMotorConfiguration.XConfiguration) {

                // calculate altitude control
                _altitudeForce = AltitudeHoldPID.Update(Altimeter.AltitudeMeters, Time.deltaTime);

                // forward right motor
                Motors[0].globalPosition = BaseRigidBody.transform.TransformPoint(new Vector3(RotorDistanceFromCenter, 0, RotorDistanceFromCenter));
                Motors[1].globalPosition = BaseRigidBody.transform.TransformPoint(new Vector3(-RotorDistanceFromCenter, 0, RotorDistanceFromCenter));
                Motors[2].globalPosition = BaseRigidBody.transform.TransformPoint(new Vector3(RotorDistanceFromCenter, 0, -RotorDistanceFromCenter));
                Motors[3].globalPosition = BaseRigidBody.transform.TransformPoint(new Vector3(-RotorDistanceFromCenter, 0, -RotorDistanceFromCenter)); ;


                Vector3 forwardRightForce = BaseRigidBody.transform.up * _altitudeForce;

                // forward left motor
                Vector3 forwardLeftForce = BaseRigidBody.transform.up * _altitudeForce;

                // back right motor
                Vector3 backRightForce = BaseRigidBody.transform.up * _altitudeForce;

                // back left motor
                Vector3 backLeftForce = BaseRigidBody.transform.up * _altitudeForce;

                Motors[0].globalForceVector += forwardRightForce;
                Motors[1].globalForceVector += forwardLeftForce;
                Motors[2].globalForceVector += backRightForce;
                Motors[3].globalForceVector += backLeftForce;

                // pitch control
                _pitchForce = PitchControllPID.Update(IMU.OrientationEulerDegrees.x, Time.deltaTime);
                forwardRightForce = BaseRigidBody.transform.up * -_pitchForce;
                forwardLeftForce = BaseRigidBody.transform.up * -_pitchForce;
                backRightForce = BaseRigidBody.transform.up * _pitchForce;
                backLeftForce = BaseRigidBody.transform.up * _pitchForce;

                Motors[0].globalForceVector += forwardRightForce;
                Motors[1].globalForceVector += forwardLeftForce;
                Motors[2].globalForceVector += backRightForce;
                Motors[3].globalForceVector += backLeftForce;


                // roll control
                _rollForce = RollControllPID.Update(IMU.OrientationEulerDegrees.z, Time.deltaTime);
                forwardRightForce = BaseRigidBody.transform.up * _rollForce;
                forwardLeftForce = BaseRigidBody.transform.up * -_rollForce;
                backRightForce = BaseRigidBody.transform.up * _rollForce;
                backLeftForce = BaseRigidBody.transform.up * -_rollForce;

                Motors[0].globalForceVector += forwardRightForce;
                Motors[1].globalForceVector += forwardLeftForce;
                Motors[2].globalForceVector += backRightForce;
                Motors[3].globalForceVector += backLeftForce;



            } else if (QuadCopterConfiguration == ZOQuadCopterMotorConfiguration.CrossConfiguration) {
                //TODO:
            }

            // apply all the forces
            foreach (Motor motor in Motors) {
                BaseRigidBody.AddForceAtPosition(motor.globalForceVector, motor.globalPosition);
            }

        }



        private void OnGUI() {
            int y = 10;
            int yInc = 21;
            GUI.TextField(new Rect(10, y += yInc, 300, 22), $"Altitude Set Point: {AltitudeHoldPID.SetPoint.ToString("n2")}");
            GUI.TextField(new Rect(10, y += yInc, 300, 22), $"Altitude: {Altimeter.AltitudeMeters.ToString("n2")}");
            GUI.TextField(new Rect(10, y += yInc, 300, 22), $"Altitude Throttle: {_altitudeForce.ToString("n2")}");

            GUI.TextField(new Rect(10, y += yInc, 300, 22), $"Orientation: {IMU.OrientationEulerDegrees.x.ToString("n2")} {IMU.OrientationEulerDegrees.y.ToString("n2")} {IMU.OrientationEulerDegrees.z.ToString("n2")}");
            GUI.TextField(new Rect(10, y += yInc, 300, 22), $"Pitch Force: {_pitchForce.ToString("n2")}");

            foreach (Motor motor in Motors) {
                Debug.DrawRay(motor.globalPosition, motor.globalForceVector, Color.green, 0.1f);
            }

        }
    }
}