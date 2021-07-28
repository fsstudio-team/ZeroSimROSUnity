using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO {
    public class ZOSpotJoystickCmdVel : MonoBehaviour {

        public ZOSpotCharacterController _spotCharacterController;

        // Start is called before the first frame update
        void Start() {

        }

        // Update is called once per frame
        void Update() {

            if (_spotCharacterController != null) {
                Vector2 targetVelocity = new Vector2(Input.GetAxis("Roll") * _spotCharacterController.MaxSideMoveVelocity, Input.GetAxis("Pitch") * _spotCharacterController.MaxForwardVelocity);
                _spotCharacterController.Move(targetVelocity, Input.GetAxis("Yaw") * _spotCharacterController._maxTurnVelocityDegreesSecond); // hack no turning yet

            }

        }
    }

}
