using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.Util {

    [RequireComponent(typeof(Rigidbody))]
    public class ZOMoveTest : MonoBehaviour {

        public Vector2 _minMax = new Vector2(0, 1.0f);
        public float _speed = 0.0f;
        // public float _time = 1.0f;

        public enum MoveTypeEnum {
            RotateRollRight,
            RotatePitchForward,
            RotateYawRight,
            TranslateForward,
            TranslateRight,
            TranslateUp
        }

        public MoveTypeEnum _moveType = MoveTypeEnum.RotateRollRight;


        private Rigidbody _rigidBody;
        private float _currentValue = 0;
        private bool _isRunning = false;
        private Vector3 _startRotationEuler = new Vector3();
        private Vector3 _startPosition = new Vector3();


        // Start is called before the first frame update
        void Start() {
            _rigidBody = GetComponent<Rigidbody>();


        }

        void Update() {
            if (Input.GetKeyDown(UnityEngine.KeyCode.Space) && _isRunning == false) {
                _currentValue = _minMax.x;
                _isRunning = true;
                _startRotationEuler = transform.rotation.eulerAngles;
                _startPosition = transform.position;
            }
        }

        // Update is called once per frame
        void FixedUpdate() {
            if (_isRunning) {
                if (_moveType == MoveTypeEnum.RotateRollRight) {
                    Quaternion rotation = Quaternion.Euler(_startRotationEuler + new Vector3(0, 0, -_currentValue));
                    _rigidBody.MoveRotation(rotation);
                } else if (_moveType == MoveTypeEnum.RotatePitchForward) {
                    Quaternion rotation = Quaternion.Euler(_startRotationEuler + new Vector3(_currentValue, 0, 0));
                    _rigidBody.MoveRotation(rotation);
                } else if (_moveType == MoveTypeEnum.RotateYawRight) {
                    Quaternion rotation = Quaternion.Euler(_startRotationEuler + new Vector3(0, _currentValue, 0));
                    _rigidBody.MoveRotation(rotation);
                } else if (_moveType == MoveTypeEnum.TranslateForward) {
                    Vector3 velocity = new Vector3(0, 0, _currentValue);
                    _rigidBody.MovePosition(_startPosition + velocity);
                } else if (_moveType == MoveTypeEnum.TranslateRight) {
                    Vector3 velocity = new Vector3(_currentValue, 0, 0);
                    _rigidBody.MovePosition(_startPosition + velocity);
                } else if (_moveType == MoveTypeEnum.TranslateUp) {
                    Vector3 velocity = new Vector3(0, _currentValue, 0);
                    _rigidBody.MovePosition(_startPosition + velocity);
                }

                _currentValue = _currentValue + (_speed * Time.deltaTime);

                if (_currentValue > _minMax.y) {
                    _isRunning = false;
                }

            }

        }
    }
}