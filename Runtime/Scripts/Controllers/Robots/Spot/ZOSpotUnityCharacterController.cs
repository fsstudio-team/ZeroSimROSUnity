using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace ZO {
    /// <summary>
    /// Handles all the Unity specific "Character Control" stuff like movement and animation.
    /// </summary>
    public class ZOSpotUnityCharacterController : MonoBehaviour {

        public Rigidbody _torsoRigidBody;
        public Transform _frontCollider = null;

        public CapsuleCollider FrontColliderCapsule {
            get {
                return _frontCollider?.GetComponent<CapsuleCollider>();
            }
        }

        public Rigidbody FrontColliderRigidBody {
            get {
                return _frontCollider?.GetComponent<Rigidbody>();
            }
        }

        public ZOGroundDetector FrontGroundDetector {
            get {
                return _frontCollider?.GetComponent<ZOGroundDetector>();
            }
        }
        public Transform _rearCollider = null;
        public CapsuleCollider RearColliderCapsule {
            get {
                return _rearCollider?.GetComponent<CapsuleCollider>();
            }
        }

        public ZOGroundDetector RearGroundDetector {
            get {
                return _rearCollider?.GetComponent<ZOGroundDetector>();
            }
        }

        public Rigidbody RearColliderRigidBody {
            get {
                return _rearCollider?.GetComponent<Rigidbody>();
            }
        }


        public float _maxFloorAngle = 45;

        public float MaxFloorAngle {
            get => _maxFloorAngle;
            set => _maxFloorAngle = value;
        }
        public float _maxForwardVelocity = 1.0f;

        public float MaxForwardVelocity {
            get => _maxForwardVelocity;
            set => _maxForwardVelocity = value;
        }
        public float _maxSideMoveVelocity = 0.5f;
        public float MaxSideMoveVelocity {
            get => _maxSideMoveVelocity;
            set => _maxSideMoveVelocity = value;
        }

        public float _maxTurnVelocityDegreesSecond = 45.0f;
        public float MaxTurnVelocityDegreesSecond {
            get => _maxTurnVelocityDegreesSecond;
            set => _maxTurnVelocityDegreesSecond = value;
        }

        private Vector2 _targetVelocity = Vector2.zero;
        private float _targetTurnVelocityDegreesPerSecond = 0.0f;

        private float _lieDownScale = 0.5f;

        private bool IsFrontGrounded {
            get {
                return FrontGroundDetector.IsGrounded;
            }
        }

        private bool IsRearGrounded {
            get {
                return RearGroundDetector.IsGrounded;
            }
        }


        private Vector3 FrontGroundNormal {
            get {
                return FrontGroundDetector.GroundNormal;
            }
        }


        private Vector3 RearGroundNormal {
            get {
                return RearGroundDetector.GroundNormal;
            }
        }



        public enum StateEnum {
            Off,
            Sitting,
            UpAndReady,
            Busy,
            Error
        }

        public ZOSpotUnityCharacterController.StateEnum State {
            get;
            private set;
        } = ZOSpotUnityCharacterController.StateEnum.Off;

        public enum ErrorEnum {
            Ok,
            GeneralError,
            ErrorShutdown
        }

        public ZOSpotUnityCharacterController.ErrorEnum Error {
            get;
            private set;
        } = ZOSpotUnityCharacterController.ErrorEnum.Ok;



        public delegate void SpotControllerStatusDelegate(ZOSpotUnityCharacterController thisClass, Vector2 linearVelocity, float turnVelocityDegreesPerSecond, float orientationDegrees, ZOSpotUnityCharacterController.StateEnum state, ZOSpotUnityCharacterController.ErrorEnum error);
        private event SpotControllerStatusDelegate _spotControllerStatusDelegate;
        public event SpotControllerStatusDelegate SpotControllerStatus {
            add {
                _spotControllerStatusDelegate += value;
            }
            remove {
                _spotControllerStatusDelegate -= value;
            }
        }



        // Start is called before the first frame update
        void Start() {
            // start sitting down
            Sit();
        }

        // Update is called once per frame
        void Update() {

        }



        void FixedUpdate() {

            if (IsFrontGrounded || IsRearGrounded) {

                Vector3 angularVelocity = _torsoRigidBody.angularVelocity;
                float torqueYaw = (_targetTurnVelocityDegreesPerSecond * Mathf.Deg2Rad) - angularVelocity.y;
                _torsoRigidBody.AddRelativeTorque(new Vector3(0, torqueYaw, 0), ForceMode.VelocityChange);

                Vector3 localVelocity = _torsoRigidBody.transform.InverseTransformDirection(_torsoRigidBody.velocity);
                Vector3 targetVelocity = new Vector3(_targetVelocity.x, 0, _targetVelocity.y);
                Vector3 deltaVelocity = targetVelocity - localVelocity;
                deltaVelocity = _torsoRigidBody.transform.TransformDirection(deltaVelocity);
                _torsoRigidBody.AddForce(deltaVelocity, ForceMode.VelocityChange);


            }


            if (_spotControllerStatusDelegate != null) {
                _spotControllerStatusDelegate.Invoke(this, new Vector2(_torsoRigidBody.velocity.x, _torsoRigidBody.velocity.z), _torsoRigidBody.angularVelocity.y * Mathf.Rad2Deg, transform.rotation.eulerAngles.y, State, Error);

            }

            // obey gravity
            if (IsFrontGrounded == true) {
                Vector3 gravity = -FrontGroundNormal * UnityEngine.Physics.gravity.magnitude * 20.2f;
                FrontColliderRigidBody.AddForce(gravity, ForceMode.Acceleration);
            } else {
                FrontColliderRigidBody.AddForce(UnityEngine.Physics.gravity * 2.2f, ForceMode.Acceleration);
            }

            if (IsRearGrounded == true) {
                Vector3 gravity = -RearGroundNormal * UnityEngine.Physics.gravity.magnitude * 20.2f;
                RearColliderRigidBody.AddForce(gravity, ForceMode.Acceleration);
            } else {
                RearColliderRigidBody.AddForce(UnityEngine.Physics.gravity * 2.2f, ForceMode.Acceleration);
            }

            // if (IsFrontGrounded == false || IsRearGrounded == false) {
            //     _rigidBody.AddForce(UnityEngine.Physics.gravity, ForceMode.Acceleration);
            // }

        }




        public void TurnOn() {

        }

        public void Stand() {
            if (State == ZOSpotUnityCharacterController.StateEnum.Sitting) {
                FrontColliderCapsule.height = FrontColliderCapsule.height * (1.0f / _lieDownScale);
                RearColliderCapsule.height = RearColliderCapsule.height * (1.0f / _lieDownScale);
                State = ZOSpotUnityCharacterController.StateEnum.UpAndReady;
            }
        }

        public void Sit() {
            if (State == ZOSpotUnityCharacterController.StateEnum.UpAndReady || State == ZOSpotUnityCharacterController.StateEnum.Off) {
                FrontColliderCapsule.height = FrontColliderCapsule.height * _lieDownScale;
                RearColliderCapsule.height = RearColliderCapsule.height * _lieDownScale;
                State = ZOSpotUnityCharacterController.StateEnum.Sitting;
            }

        }


        public void Move(Vector2 targetVelocity, float targetTurnVelocityDegreesPerSecond) {
            if (State == ZOSpotUnityCharacterController.StateEnum.UpAndReady) {
                _targetVelocity = targetVelocity;
                _targetTurnVelocityDegreesPerSecond = targetTurnVelocityDegreesPerSecond;
            }
        }
        // void Sit(Delegate OnSitFinishedOrError)
        // void Stand(Delegate OnStandFinishedOrError)
        // void TurnOn(Delegate OnTurnOnFinishedOrError)
        // void TurnOff(Delegate OnTurnOffFinishedOrError)
    }

}
