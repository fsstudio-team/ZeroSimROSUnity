using System.Xml;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NWH;

namespace ZO {
    public class ZOGenericForkliftControllerWheel : MonoBehaviour {

        [Header("Rigid Bodies")]
        public Rigidbody _chassisRigidBody;

        [Header("Wheel Controllers")]
        public NWH.WheelController3D.WheelController _rightFrontWheel;
        public NWH.WheelController3D.WheelController _leftFrontWheel;
        public NWH.WheelController3D.WheelController _rightRearWheel;
        public NWH.WheelController3D.WheelController _leftRearWheel;

        [Header("Mast Control")]
        public ZO.Physics.ZOLinearActuator _mastLinearActuator;
        public float _mastRiseSpeed = 0.01f;
        public ZO.Physics.ZOServoMotorActuator _mastTiltController;
        public float _mastTiltSpeed = 1.0f;

        [Header("Contol Limits")]
        public float _maxSteeringAngle = 35;
        public float _minSteeringAngle = 20;

        public float _maxMotorTorque;
        public float _maxWheelRPM;
        public float _maxBrakeTorque;

        [Header("Drive Motor Control")]
        public ZO.Physics.ZOPIDController _driveMotorPID;



        // Start is called before the first frame update
        void Start() {
            _driveMotorPID._maximumOutputValue = _maxMotorTorque;
        }



        // Update is called once per frame
        void Update() {

        }

        private float _smoothAxis = 0;
        private float _xAxisVelocity = 0;
        private void FixedUpdate() {


            // ~~~~~~~ Driving ~~~~~~~ //
            float steerInput = Input.GetAxis("Horizontal");
            float driveInput = Input.GetAxis("Vertical");

            float velocity = transform.InverseTransformDirection(_chassisRigidBody.velocity).z;
            _smoothAxis = Mathf.SmoothDamp(_smoothAxis, steerInput, ref _xAxisVelocity, 0.12f);


            // apply steering
            float steerAngle = Mathf.Lerp(_maxSteeringAngle, _minSteeringAngle, Mathf.Abs(velocity) * 0.05f) * steerInput;
            _leftRearWheel.steerAngle = steerAngle;
            _rightRearWheel.steerAngle = steerAngle;

            // apply drive motor
            _driveMotorPID.SetPoint = driveInput * _maxWheelRPM;
            float motorTorque = 0;
            float brakeTorque = 0;
            if (Mathf.Abs(_driveMotorPID.SetPoint) > 0.1 * _maxWheelRPM) {  // apply drive
                float currentAverageWheelRPM = (_leftFrontWheel.rpm + _rightFrontWheel.rpm) * 0.5f;
                motorTorque = _driveMotorPID.Update(currentAverageWheelRPM, Time.fixedDeltaTime);
                brakeTorque = 0;
            } else { // auto braking
                motorTorque = 0;
                brakeTorque = _maxBrakeTorque;
                // Debug.Log("Brake");
            }

            _leftFrontWheel.motorTorque = motorTorque;
            _rightFrontWheel.motorTorque = motorTorque;

            _leftFrontWheel.brakeTorque = brakeTorque;
            _rightFrontWheel.brakeTorque = brakeTorque;
            _rightRearWheel.brakeTorque = brakeTorque;
            _leftRearWheel.brakeTorque = brakeTorque;

            // ~~~~~~~ Mast Tilt Control ~~~~~~~~~ //
            if (Input.GetKey(KeyCode.Comma)) {
                _mastTiltController.TargetAngleDegrees -= _mastTiltSpeed;
            }

            if (Input.GetKey(KeyCode.Period)) {
                _mastTiltController.TargetAngleDegrees += _mastTiltSpeed;
            }

            // ~~~~~~~~ Mast Linear Actuation ~~~~~~~~~ //
            if (Input.GetButton("Fire1")) {
                _mastLinearActuator.SetPosition += _mastRiseSpeed;
            }

            if (Input.GetButton("Fire2")) {
                _mastLinearActuator.SetPosition -= _mastRiseSpeed;
            }


        }
    }
}
