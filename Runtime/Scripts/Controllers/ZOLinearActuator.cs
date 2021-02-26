using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ZO.Physics;

namespace ZO.Controllers {
    public class ZOLinearActuator : MonoBehaviour {

        public ConfigurableJoint _prismaticJoint;

        public ZOPIDController _pidController;
        public enum Axis { X = 0, Y = 1, Z = 2 }
        public Axis _driveAxis;


        public float SetPosition {
            set {_pidController.SetPoint = value; }
            get { return _pidController.SetPoint; }
        }

        // Start is called before the first frame update
        void Start() {

        }

        // Update is called once per frame
        void Update() {

        }

        private void FixedUpdate() {
            Rigidbody slideBody = _prismaticJoint.gameObject.GetComponent<Rigidbody>();
            float currentPosition = slideBody.transform.localPosition[(int)_driveAxis];
            float force = _pidController.Update(currentPosition, Time.deltaTime);
            Vector3 globalUpForce = new Vector3(0, 0, 0);
            globalUpForce[(int)_driveAxis] = force;
            Vector3 localUpForce = slideBody.transform.worldToLocalMatrix.MultiplyVector(globalUpForce);
            slideBody.AddRelativeForce(localUpForce, ForceMode.Force);

            // add equal and opposite force
            Rigidbody opposingBody = _prismaticJoint.connectedBody;
            localUpForce = -1.0f * localUpForce; // flip
            opposingBody.AddRelativeForce(localUpForce, ForceMode.Force);

        }
    }
}