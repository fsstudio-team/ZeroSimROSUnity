using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.Util {

    [RequireComponent(typeof(Rigidbody))]
    public class ZORandomizePhysics : MonoBehaviour {

        [Header("Randomize Rotation")]
        public Vector3 _startAngularVelocityDegreesPerSecond = new Vector3(0,0,0);

        private Rigidbody _rigidBody;


        // Start is called before the first frame update
        void Start() {
            _rigidBody = GetComponent<Rigidbody>();

            // set starting angular velocity
            Vector3 startAngularVelocityRadiansPerSecond = _startAngularVelocityDegreesPerSecond;
            startAngularVelocityRadiansPerSecond.x = startAngularVelocityRadiansPerSecond.x * Mathf.Deg2Rad;
            startAngularVelocityRadiansPerSecond.y = startAngularVelocityRadiansPerSecond.y * Mathf.Deg2Rad;
            startAngularVelocityRadiansPerSecond.z = startAngularVelocityRadiansPerSecond.z * Mathf.Deg2Rad;
            _rigidBody.angularVelocity = startAngularVelocityRadiansPerSecond;
        }

        // Update is called once per frame
        void Update() {
        }
    }
}