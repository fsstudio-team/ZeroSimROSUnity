using System.Collections.Generic;
using UnityEngine;

namespace ZO {

    public class ZOGroundDetector : MonoBehaviour {

        public float _maxFloorAngle = 45;
        public bool _debug = false;

        public float MaxFloorAngle {
            get => _maxFloorAngle;
            set => _maxFloorAngle = value;
        }

        public CapsuleCollider CapsuleCollider {
            get {
                return GetComponent<CapsuleCollider>();
            }
        }

        bool _isGrounded = false;
        public bool IsGrounded {
            get {
                return _isGrounded;
            }
            private set {
                _isGrounded = value;
            }
        }

        Vector3 _groundNormal = -UnityEngine.Physics.gravity.normalized;
        public Vector3 GroundNormal {
            get => _groundNormal;
            private set => _groundNormal = value;
        }

        Vector3 _groundPoint = Vector3.zero;
        public Vector3 GroundPoint {
            get => _groundPoint;
            private set => _groundPoint = value;
        }

        // void OnCollisionEnter(Collision collision) {
        //     IsGrounded = CheckIsGrounded(collision, CapsuleCollider, out _groundNormal);
        // }

        void OnCollisionStay(Collision collision) {
            IsGrounded = CheckIsGrounded(collision);
        }

        void OnCollisionExit(Collision collision) {
            IsGrounded = CheckIsGrounded(collision);
        }

        bool CheckIsGrounded(Collision collision) {
            foreach (ContactPoint contactPoint in collision.contacts) {
                // if (contactPoint.thisCollider == capsuleCollider) {
                if (MaxFloorAngle > Vector3.Angle(contactPoint.normal, -UnityEngine.Physics.gravity.normalized)) {
                    GroundNormal = contactPoint.normal;
                    GroundPoint = contactPoint.point;
                    
                    // Debug.DrawRay(contactPoint.point, contactPoint.normal * 100, Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f), 3f);

                    return true;
                }
                // }
            }

            GroundNormal = -UnityEngine.Physics.gravity.normalized;

            // foreach (var item in collision.contacts) {
            //     Debug.DrawRay(item.point, item.normal * 100, Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f), 10f);
            // }


            return false;
        }

        public void OnGUI() {
            if (_debug) {
                if (IsGrounded) {
                    GUI.TextField(new Rect(10, 10, 500, 22), "On Ground");
                } else {
                    GUI.TextField(new Rect(10, 10, 500, 22), "Off Ground");
                }

                // Draw a different colored ray for every normal in the collision

            }
        }


    }
}