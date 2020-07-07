using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.Controllers {

    [System.Serializable]
    public class _Wheel {
        public NWH.WheelController3D.WheelController wheelController;
        public bool steer;
        public bool power;
    }

    public class ZOGenericRCCarController : MonoBehaviour {

        [Header("General")]
        public bool _isUserControlled = true;

        [Header("Rigid Bodies")]
        public Rigidbody _chassisRigidBody;

        [SerializeField]
        public List<_Wheel> _wheelControllers;



        [Header("Motor Control")]
        public float _maxMotorTorque;
        public float _maxBrakeTorque;
        public float _antiRollBarForce;

        [Header("Steering Control")]
        public ZO.Physics.ZOPIDController _steeringPid;


        private float _steeringInput = 0;
        private float _driveInput = 0;

        public bool IsUserControlled {
            get { return _isUserControlled; }
            set { _isUserControlled = value; }
        }

        public Vector3 Position {
            get { return _chassisRigidBody.position; }
            set { _chassisRigidBody.position = value; }
        }

        public Quaternion Orientation {
            get { return _chassisRigidBody.rotation; }
            set { _chassisRigidBody.rotation = value; }
        }

        public Vector3 Velocity {
            get { return _chassisRigidBody.velocity; }
            set { _chassisRigidBody.velocity = value; }
        }

        public Vector3 AngularVelocity {
            get { return _chassisRigidBody.angularVelocity; }
            set { _chassisRigidBody.angularVelocity = value; }
        }

        public Vector3 Forward {
            get { return _chassisRigidBody.transform.forward; }
        }

        public float DriveInput {
            get { return _driveInput; }
            set { _driveInput = value; }
        }

        public float SteeringInput {
            get { return _steeringInput; }
            set { _steeringInput = value; }
        }

        public float WheelRPM(int wheel) {
            return _wheelControllers[wheel].wheelController.rpm;
        }
        // Start is called before the first frame update
        void Start() {

        }

        // Update is called once per frame
        void Update() {
            if (_isUserControlled == true) {
                _steeringInput = Input.GetAxis("Horizontal");
                _driveInput = Input.GetAxis("Vertical");
            }

            _steeringPid.SetPoint = _steeringPid.MaximumOutputValue * _steeringInput;

        }
 
        public void FixedUpdate() {
            foreach (_Wheel wheel in _wheelControllers) {
                if (Input.GetKey(KeyCode.Space)) {
                    wheel.wheelController.brakeTorque = _maxBrakeTorque;
                } else {
                    wheel.wheelController.brakeTorque = 0.0f;
                }

                // if (Mathf.Sign(_velocity) < 0.01f && _inputYAxis > 0.1f)
                //     w.wc.brakeTorque = _maxBrakeTorque;

                if (wheel.power) {
                    wheel.wheelController.motorTorque = _maxMotorTorque * _driveInput;
                }

                if (wheel.steer) {

                    wheel.wheelController.steerAngle = _steeringPid.Update(wheel.wheelController.steerAngle, Time.fixedDeltaTime);
                }
            }
            // ApplyAntirollBar(); 
        }
    
        public void ApplyAntirollBar() {
            for (int i = 0; i < _wheelControllers.Count; i += 2) {
                NWH.WheelController3D.WheelController leftWheel = _wheelControllers[i].wheelController;
                NWH.WheelController3D.WheelController rightWheel = _wheelControllers[i + 1].wheelController;

                if (!leftWheel.springOverExtended && !leftWheel.springBottomedOut && !rightWheel.springOverExtended && !rightWheel.springBottomedOut) {
                    float leftTravel = leftWheel.springTravel;
                    float rightTravel = rightWheel.springTravel;

                    float arf = (leftTravel - rightTravel) * _antiRollBarForce;

                    if (leftWheel.isGrounded)
                        leftWheel.parent.GetComponent<Rigidbody>().AddForceAtPosition(leftWheel.wheel.up * -arf, leftWheel.wheel.worldPosition);

                    if (rightWheel.isGrounded)
                        rightWheel.parent.GetComponent<Rigidbody>().AddForceAtPosition(rightWheel.wheel.up * arf, rightWheel.wheel.worldPosition);
                }
            }
        }



    }
}