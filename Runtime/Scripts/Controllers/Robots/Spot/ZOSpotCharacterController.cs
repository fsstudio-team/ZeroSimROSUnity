using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace ZO {
    /// <summary>
    /// Handles all the Unity specific "Character Control" stuff like movement and animation.
    /// </summary>
    public class ZOSpotCharacterController : MonoBehaviour {

        public Rigidbody _rigidBody;
        public Transform _frontCollider = null;
        public Transform _rearCollider = null;
        public float _maxFloorAngle = 45;
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

        public enum StateEnum {
            Off,
            Down,
            UpAndReady,
            Error
        }

        public ZOSpotCharacterController.StateEnum State {
            get;
            private set;
        } = ZOSpotCharacterController.StateEnum.Off;

        public enum ErrorEnum {
            Ok,
            GeneralError,
            ErrorShutdown
        }

        public ZOSpotCharacterController.ErrorEnum Error {
            get;
            private set;
        } = ZOSpotCharacterController.ErrorEnum.Ok;



        public delegate void SpotControllerStatusDelegate(ZOSpotCharacterController thisClass, Vector2 linearVelocity, float turnVelocityDegreesPerSecond, float orientationDegrees, ZOSpotCharacterController.StateEnum state, ZOSpotCharacterController.ErrorEnum error);
        private event SpotControllerStatusDelegate _spotContrllerStatusDelegate;
        public event SpotControllerStatusDelegate SpotControllerStatus {
            add {
                _spotContrllerStatusDelegate += value;
            }
            remove {
                _spotContrllerStatusDelegate -= value;
            }
        }



        // Start is called before the first frame update
        void Start() {

        }

        // Update is called once per frame
        void Update() {

        }

        void FixedUpdate() {

            _rigidBody.velocity = _rigidBody.transform.rotation * new Vector3(_targetVelocity.x, _rigidBody.velocity.y, _targetVelocity.y);

            Vector3 angularVelocity = _rigidBody.angularVelocity;
            angularVelocity.y = _targetTurnVelocityDegreesPerSecond * Mathf.Deg2Rad;
            _rigidBody.angularVelocity = angularVelocity;

            if (_spotContrllerStatusDelegate != null) {
                _spotContrllerStatusDelegate.Invoke(this, new Vector2(_rigidBody.velocity.x, _rigidBody.velocity.z), _rigidBody.angularVelocity.y * Mathf.Rad2Deg, transform.rotation.eulerAngles.y, State, Error);

            }
        }


        public void Move(Vector2 targetVelocity, float targetTurnVelocityDegreesPerSecond) {
            _targetVelocity = targetVelocity;
            _targetTurnVelocityDegreesPerSecond = targetTurnVelocityDegreesPerSecond;
        }
        // void Sit(Delegate OnSitFinishedOrError)
        // void Stand(Delegate OnStandFinishedOrError)
        // void TurnOn(Delegate OnTurnOnFinishedOrError)
        // void TurnOff(Delegate OnTurnOffFinishedOrError)
    }

}
