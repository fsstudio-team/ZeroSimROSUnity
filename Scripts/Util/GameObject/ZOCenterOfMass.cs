using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.Util {
    [RequireComponent(typeof(Rigidbody))]
    [ExecuteInEditMode]
    public class ZOCenterOfMass : MonoBehaviour {
        private Vector3 _centerOfMass;
        public Vector3 _centerOfMassOffset = Vector3.zero;
        private Vector3 _prevOffset = Vector3.zero;
        private Rigidbody _rigidBody;
        public bool _debugShowCenterOfMass = true;

        void Start() {
            _rigidBody = GetComponent<Rigidbody>();
            _centerOfMass = _rigidBody.centerOfMass;
        }


        void Update() {
            if (_centerOfMassOffset != _prevOffset) {
                _rigidBody.centerOfMass = _centerOfMass + _centerOfMassOffset;
            }
            _prevOffset = _centerOfMassOffset;
        }

        private void OnDrawGizmos() {
            if (_debugShowCenterOfMass && _rigidBody != null) {
                float radius = 0.1f;
                try {
                    radius = GetComponent<MeshFilter>().sharedMesh.bounds.size.z / 10f;
                } catch { }

                Vector3 centerOfMass = _rigidBody.transform.TransformPoint(_rigidBody.centerOfMass);
                Gizmos.color = Color.green;
                Gizmos.DrawLine(centerOfMass + new Vector3(0, -radius, 0), centerOfMass + new Vector3(0, radius, 0));
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(centerOfMass + new Vector3(0, 0, -radius), centerOfMass + new Vector3(0, 0, radius));
                Gizmos.color = Color.red;
                Gizmos.DrawLine(centerOfMass + new Vector3(-radius, 0, 0), centerOfMass + new Vector3(radius, 0, 0));

                Gizmos.color = Color.white;
                Gizmos.DrawSphere(_rigidBody.transform.TransformPoint(_rigidBody.centerOfMass), radius/4.0f);
            }
        }
    }
}

