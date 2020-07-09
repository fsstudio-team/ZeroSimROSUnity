using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using Newtonsoft.Json.Linq;
using ZO.Physics;
using ZO.Util.Extensions;

namespace ZO.Controllers {


    /// <summary>
    /// A PID controlled servo motor controller.
    /// </summary>
    [ExecuteAlways]
    public class ZOServoMotorActuator : MonoBehaviour, ZO.ZOSimTypeInterface {

        public string _name;
        public ZOHingeJoint _hingeJoint;
        public ZOPIDController _pidController;
        public float _maxForce = 2000;
        [SerializeField] [ZO.Util.ZOReadOnly] float _currentAngleDegrees;

        /// <summary>
        /// Read-only get max steering lock in degrees.
        /// 
        /// </summary>
        /// <value></value>
        public float MaxSteeringLockDegrees {
            get { return UnityHingeJoint.limits.max; }
            set {
                JointLimits limits = UnityHingeJoint.limits;
                limits.max = value;
                UnityHingeJoint.limits = limits;
            }
        }

        public float MinSteeringLockDegrees {
            get { return UnityHingeJoint.limits.min; }
            set {
                JointLimits limits = UnityHingeJoint.limits;
                limits.min = value;
                UnityHingeJoint.limits = limits;
            }
        }

        /// <summary>
        /// Get/Set steering angle degrees
        /// </summary>
        /// <value></value>
        public float TargetAngleDegrees {
            get { return _pidController.SetPoint; }
            set {
                _pidController.SetPoint = Mathf.Clamp(value, MinSteeringLockDegrees, MaxSteeringLockDegrees);
            }
        }

        public float CurrentAngleDegrees {
            get { return UnityHingeJoint.angle; }
        }

        public HingeJoint UnityHingeJoint {
            get { return _hingeJoint.UnityHingeJoint; }
        }

        //~~~ ZOSimTypeInterface ~~~//
        public string Type {
            get { return "controller.servor_motor_actuator"; }
        }
        public string Name {
            get {
                return _name;
            }
            private set { _name = value; }
        }

        private ZOSimDocumentRoot _documentRoot = null;
        private JObject _json;
        public JObject JSON {
            get {
                // if (_json == null) {
                //     _json = BuildJSON();
                // }
                return _json;
            }
        }

        public void ImportZeroSim(ZOSimDocumentRoot documentRoot, JObject json) {
            throw new System.NotImplementedException("TODO!");
        }

        public JObject BuildJSON(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject json = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("hinge_joint", _hingeJoint.Name),
                new JProperty("min_limit_degrees", MinSteeringLockDegrees),
                new JProperty("max_limit_degrees", MaxSteeringLockDegrees),
                new JProperty("pid_controller", _pidController.BuildJSON(documentRoot))

            );
            _json = json;
            return json;
        }

        public void LoadFromJSON(ZOSimDocumentRoot documentRoot, JObject json) {
            // Assert.Equals(json["type"].Value<string>() == Type);

            _json = json;
            Name = json.ValueOrDefault("name", Name);

            // TODO: build Hinge Joint if it doesn't exist?

            MinSteeringLockDegrees = json.ValueOrDefault<float>("min_limit_degrees", MinSteeringLockDegrees);
            MaxSteeringLockDegrees = json.ValueOrDefault<float>("max_limit_degrees", MaxSteeringLockDegrees);
            if (_pidController == null) {
                _pidController = new ZOPIDController();
            }
            _pidController.LoadFromJSON(documentRoot, json["pid_controller"].Value<JObject>());

        }


        private void Start() {
            if (Application.IsPlaying(gameObject) == false) { // In Editor Mode 
                if (_name.Length == 0) {
                    _name = Type;
                    if (_hingeJoint != null) {
                        _name = _name + "_" + _hingeJoint.Name;
                    }
                    _pidController._name = _pidController.Type + "_" + _name;
                }
            }
        }


        private void FixedUpdate() {

            float motorTargetVelocity = _pidController.Update(UnityHingeJoint.angle, Time.fixedDeltaTime);
            if (_pidController.IsAtDeadBand) {
                JointSpring hingeJointSpring = UnityHingeJoint.spring;
                hingeJointSpring.damper = 50000.0f;
                hingeJointSpring.spring = 1e+08f;
                hingeJointSpring.targetPosition = UnityHingeJoint.angle;
                UnityHingeJoint.spring = hingeJointSpring;
                UnityHingeJoint.useSpring = true;
                UnityHingeJoint.useMotor = false;

            } else { // drive to position
                JointMotor motor = UnityHingeJoint.motor;
                motor.targetVelocity = motorTargetVelocity;
                motor.force = _maxForce;
                motor.freeSpin = false;
                UnityHingeJoint.motor = motor;
                UnityHingeJoint.useMotor = true;
                UnityHingeJoint.useSpring = false;

            }

            // Debug.Log("Joint Angle: " + _hingeJoint.angle);
            // Debug.Log("Motor Force: " + motor.force);
            // if (f % 10 == 0) {
            //     Debug.Log("motor.force = " + motor.force);
            //     Debug.Log("velocity = " + _hingeJoint.velocity);
            //     Debug.Log("angle deg = " + _hingeJoint.angle);
            //     // Debug.Log("angle rad = " + (Mathf.Deg2Rad * _hingeJoint.angle));
            // }
            // f++;

        }
    }

}
