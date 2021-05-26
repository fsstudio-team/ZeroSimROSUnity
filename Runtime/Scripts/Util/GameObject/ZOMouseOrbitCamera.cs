using UnityEngine;
using System.Collections;

namespace ZO.Util {
    public class ZOMouseOrbitCamera : MonoBehaviour {

        public Transform target;
        public float _distance = 5.0f;
        public float _xSpeed = 120.0f;
        public float _ySpeed = 120.0f;

        public float _yMinLimit = -20f;
        public float _yMaxLimit = 80f;

        public float _distanceMin = .5f;
        public float _distanceMax = 15f;

        private Rigidbody _rigidbody;

        float x = 0.0f;
        float y = 0.0f;

        // Use this for initialization
        void Start() {
            Vector3 angles = transform.eulerAngles;
            x = angles.y;
            y = angles.x;

            _rigidbody = GetComponent<Rigidbody>();

            // Make the rigid body not change rotation
            if (_rigidbody != null) {
                _rigidbody.freezeRotation = true;
            }
        }

        void LateUpdate() {
            if (target) {
                if (Input.GetMouseButton(0)) {
                    x += Input.GetAxis("Mouse X") * _xSpeed * _distance * 0.02f;
                    y -= Input.GetAxis("Mouse Y") * _ySpeed * 0.02f;
                }

                y = ClampAngle(y, _yMinLimit, _yMaxLimit);

                Quaternion rotation = Quaternion.Euler(y, x, 0);

                
                float distanceDelta = _distance - (Input.GetAxis("Mouse ScrollWheel") * 5.0f);
                // Debug.Log("INFOO: Mouse scroll wheel: " + distanceDelta);
                _distance = Mathf.Clamp(distanceDelta, _distanceMin, _distanceMax);

                // RaycastHit hit;
                // if (UnityEngine.Physics.Linecast(target.position, transform.position, out hit)) {
                //     distance -= hit.distance;
                // }
                Vector3 negDistance = new Vector3(0.0f, 0.0f, -_distance);
                Vector3 position = rotation * negDistance + target.position;

                transform.rotation = rotation;
                transform.position = position;
            }
        }

        public static float ClampAngle(float angle, float min, float max) {
            if (angle < -360F)
                angle += 360F;
            if (angle > 360F)
                angle -= 360F;
            return Mathf.Clamp(angle, min, max);
        }
    }
}