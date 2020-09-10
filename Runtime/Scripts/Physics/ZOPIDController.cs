using Newtonsoft.Json.Linq;
using ZO.Util.Extensions;
using ZO.Document;

namespace ZO.Physics {

    /// <summary>
    /// Proportional Integral Derivative (PID) controller.
    /// </summary>
    [System.Serializable]
    public class ZOPIDController : ZOSerializationInterface {
        public string _name;

        [UnityEngine.Tooltip("Proportional constant (counters current error)")]
        public float _Kp;
        /// <summary>
        /// Proportional 
        /// </summary>
        /// <value></value>
        public float Kp {
            get => _Kp;
            set => _Kp = value;
        }

        [UnityEngine.Tooltip("Integral constant (counters cumulated error)")]
        public float _Ki;

        /// <summary>
        /// Integral
        /// </summary>
        /// <value></value>
        public float Ki {
            get => _Ki;
            set => _Ki = value;
        }

        [UnityEngine.Tooltip("Derivative constant (fights oscillation)")]
        public float _Kd;
        /// <summary>
        /// Derivative
        /// </summary>
        /// <value></value>
        public float Kd {
            get => _Kd;
            set => _Kd = value;
        }

        [UnityEngine.Tooltip("Set point value")]
        public float _setPoint = 0;
        /// <summary>
        /// PID set point. The value it is trying to "get" to.
        /// </summary>
        /// <value></value>
        public float SetPoint {
            get => _setPoint;
            set => _setPoint = value;
        }

        [UnityEngine.Tooltip("Maximum Output Value")]
        public float _maximumOutputValue;
        /// <summary>
        /// Maximum output value that we clamp to.
        /// </summary>
        /// <value></value>
        public float MaximumOutputValue {
            get => _maximumOutputValue;
            set => _maximumOutputValue = value;
        }

        [UnityEngine.Tooltip("Deadband")]
        public float _deadBandEpsilon = 0;

        /// <summary>
        /// Deadband area where if value jump around we consider the PID converged to the set point.
        /// </summary>
        /// <value></value>
        public float DeadBandEpsilon {
            get => _deadBandEpsilon;
            set => _deadBandEpsilon = value;
        }


        private bool _isAtDeadBand = false;

        /// <summary>
        /// Determines if we are within the deadband epsilon area. 
        /// <see>DeadBandEpsilon</see>
        /// </summary>
        /// <value></value>
        public bool IsAtDeadBand {
            get => _isAtDeadBand;
        }

        private float _lastError = 0;
        private float _integral = 0;

        [ZO.Util.ZOReadOnly] public float _output = 0;



        //~~~ ZOSimTypeInterface ~~~//
        public string Type {
            get { return "controller.pid"; }
        }
        public string Name {
            get {
                return _name;
            }
            private set { _name = value; }
        }


        ZOSimDocumentRoot _documentRoot = null;
        private JObject _json;
        public JObject JSON {
            get {
                // if (_json == null) {
                //     _json = BuildJSON(_documentRoot);
                // }
                return _json;
            }
        }


        public JObject Serialize(ZOSimDocumentRoot documentRoot, UnityEngine.Object parent = null) {
            JObject gripControllerJSON = new JObject(
                new JProperty("name", Name),
                new JProperty("type", Type),
                new JProperty("kp", _Kp),
                new JProperty("ki", _Ki),
                new JProperty("kd", _Kd),
                new JProperty("set_point", _setPoint),
                new JProperty("dead_band_epsilon", _deadBandEpsilon),
                new JProperty("maximum_output", _maximumOutputValue)

            );
            return gripControllerJSON;
        }

        public void Deserialize(ZOSimDocumentRoot documentRoot, JObject json) {
            // Assert.Equals(json["type"].Value<string>() == Type);

            _json = json;
            Name = json.ValueOrDefault("name", Name);
            Kp = json.ValueOrDefault("kp", Kp);
            Ki = json.ValueOrDefault("ki", Ki);
            Kd = json.ValueOrDefault("kd", Kd);
            SetPoint = json.ValueOrDefault("set_point", SetPoint);
            DeadBandEpsilon = json.ValueOrDefault("dead_band_epsilon", DeadBandEpsilon);
            MaximumOutputValue = json.ValueOrDefault("maximum_output", MaximumOutputValue);

        }


        public float Update(float current, float dt) {
            float error = _setPoint - current;
            if (UnityEngine.Mathf.Abs(error) <= _deadBandEpsilon) {
                error = 0.0f;
                _isAtDeadBand = true;
                _output = 0.0f;
                return 0.0f;
            }
            _isAtDeadBand = false;

            float derivative = (error - _lastError) / dt;
            _integral = _integral + error * dt;
            _lastError = error;
            float output = (_Kp * error) + (_Ki * _integral) + (_Kd * derivative);
            if (UnityEngine.Mathf.Abs(output) > _maximumOutputValue) { // clamp output
                output = UnityEngine.Mathf.Sign(output) * _maximumOutputValue;
            }
            _output = output;
            return output;
        }
    }
}
