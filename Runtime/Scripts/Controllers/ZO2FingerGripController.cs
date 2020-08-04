// using System.Numerics;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;

namespace ZO.Controllers {

    [ExecuteAlways]
    public class ZO2FingerGripController : MonoBehaviour, ZO.ZOSerializationInterface {

        [SerializeField] public string _name;
        public Vector2 _minMaxLimitsDegrees = new Vector2(0, 45);
        public float _onKeyMoveSpeed = 2.0f;
        [SerializeField] [ZO.Util.ZOReadOnly] public ZO.Controllers.ZOServoMotorActuator[] _fingerActuators = new ZO.Controllers.ZOServoMotorActuator[2];

        private float _targetAngle = 0;


        //~~~ ZOSimTypeInterface ~~~//
        public string Type {
            get { return "controller.2fingergrip"; }
        }
        public string Name {
            get {
                return _name;
            }
        }

        private JObject _json;
        public JObject JSON {
            get {
                // if (_json == null) {
                //     _json = BuildJSON();
                // }
                return _json;

            }
        }


        public JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject gripControllerJSON = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("servo_motor_actuators",
                    new JArray(_fingerActuators[0].Name, _fingerActuators[1].Name)),
                new JProperty("min_limit_degrees", _minMaxLimitsDegrees.x),
                new JProperty("max_limit_degrees", _minMaxLimitsDegrees.y),
                new JProperty("speed", _onKeyMoveSpeed)
            );
            _json = gripControllerJSON;
            return gripControllerJSON;

        }

        public void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            throw new System.NotImplementedException("TODO!");
        }


        // Start is called before the first frame update
        void Start() {
            if (Application.IsPlaying(gameObject) == false) { // In Editor Mode
                if (_name == null) {
                    _name = Type + "_" + gameObject.name;
                }
            }
            _fingerActuators[0].MinSteeringLockDegrees = _minMaxLimitsDegrees.x;
            _fingerActuators[1].MinSteeringLockDegrees = _minMaxLimitsDegrees.x;
            _fingerActuators[0].MaxSteeringLockDegrees = _minMaxLimitsDegrees.y;
            _fingerActuators[1].MaxSteeringLockDegrees = _minMaxLimitsDegrees.y;

        }

        // Update is called once per frame
        void Update() {
            if (Input.GetKey(KeyCode.UpArrow)) {
                _targetAngle += (_onKeyMoveSpeed + Time.deltaTime);
            }

            if (Input.GetKey(KeyCode.DownArrow)) {
                _targetAngle -= (_onKeyMoveSpeed + Time.deltaTime);
            }

            _targetAngle = Mathf.Clamp(_targetAngle, _minMaxLimitsDegrees.x, _minMaxLimitsDegrees.y);

            _fingerActuators[0].TargetAngleDegrees = _targetAngle;
            _fingerActuators[1].TargetAngleDegrees = _targetAngle;
        }
    }

}
