using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.Util {
    public class ZORandomizeTransform : MonoBehaviour {

        public float _translationSpeed;
        public Vector3 _translationMinRange;
        public Vector3 _translationMaxRange;

        public float _rotationSpeed;
        public Vector3 _rotationMinRange;
        public Vector3 _rotationMaxRange;


        private Vector3 _initialTranslation;
        private Vector3 _targetTranslation;
        private Quaternion _initialRotation;
        private Quaternion _targetRotation;

        // Start is called before the first frame update
        void Start() {
            _initialTranslation = transform.position;
            _targetTranslation = _initialTranslation;
            _initialRotation = transform.rotation;
            _targetRotation = _initialRotation;
        }

        // Update is called once per frame
        void Update() {
            Vector3 deltaTranslation = transform.position - _targetTranslation;
            if (deltaTranslation.magnitude <= Mathf.Epsilon) {
                deltaTranslation = new Vector3(
                    Random.Range(_translationMinRange.x, _translationMaxRange.x),
                    Random.Range(_translationMinRange.y, _translationMaxRange.y),
                    Random.Range(_translationMinRange.z, _translationMaxRange.z));
                _targetTranslation = _initialTranslation + deltaTranslation;
            }
            transform.position = Vector3.MoveTowards(transform.position, _targetTranslation, Time.deltaTime * _translationSpeed);

            if (Mathf.Abs(Quaternion.Dot(transform.rotation, _targetRotation)) <= 1.0f - Mathf.Epsilon) {
                Vector3 deltaRotationEuler = new Vector3(
                    Random.Range(_rotationMinRange.x, _rotationMaxRange.x),
                    Random.Range(_rotationMinRange.y, _rotationMaxRange.y),
                    Random.Range(_rotationMinRange.z, _rotationMaxRange.z));

                Quaternion deltaRotation = new Quaternion();
                deltaRotation.eulerAngles = deltaRotationEuler;
                _targetRotation = deltaRotation * _initialRotation;
            }

            transform.rotation = Quaternion.RotateTowards(transform.rotation, _targetRotation, _rotationSpeed);
        }
    }
}