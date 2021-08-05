using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace ZO {
    public class ZOSpotJoystickCmdVel : MonoBehaviour {

        public ZOSpotCharacterController _spotCharacterController;


        // Start is called before the first frame update
        void Start() {
        }

        // public void SitCB(InputAction.CallbackContext context) {
        //     Debug.Log("YO");
        // }

        // Update is called once per frame
        void FixedUpdate() {

            if (Input.GetAxis("Sit") > 0) {
                Debug.Log("INFO: Sit");
                _spotCharacterController.Sit();
            }

            if (Input.GetButtonDown("Stand") == true) {
                Debug.Log("INFO: Stand");
                _spotCharacterController.Stand();
            }


            if (_spotCharacterController != null) {
                Vector2 targetVelocity = new Vector2(Input.GetAxis("Roll") * _spotCharacterController.MaxSideMoveVelocity, Input.GetAxis("Pitch") * _spotCharacterController.MaxForwardVelocity);
                _spotCharacterController.Move(targetVelocity, Input.GetAxis("Yaw") * _spotCharacterController._maxTurnVelocityDegreesSecond); // hack no turning yet


            }

        }
    }

}
