using System;
namespace ZO.Physics {

    /// <summary>
    /// Velocity, Acceleration and Jerk limiter
    /// See: https://github.com/ros-controls/ros_controllers/blob/noetic-devel/diff_drive_controller/src/speed_limiter.cpp
    /// </summary>
    [System.Serializable]
    public class ZOSpeedLimiter { // TODO: ZO.ZOSimTypeInterface

        /// <summary>
        /// If true applies velocity limits
        /// </summary>
        public bool _hasVelocityLimits = false;

        /// <summary>
        /// if true, applies acceleration limits
        /// </summary>
        public bool _hasAccelerationLimits = false;

        /// <summary>
        /// if true, applies jerk limits
        /// </summary>
        public bool _hasJerkLimits = false;
        public float _minVelocity = 0;
        public float _maxVelocity = 0;
        public float _minAcceleration = 0;
        public float _maxAcceleration = 0;
        public float _minJerk = 0;
        public float _maxJerk = 0;

        /// <summary>
        /// Limit velocity and acceleration
        /// </summary>
        /// <param name="velocity">velocity [m/s]</param>
        /// <param name="velocity0">previous velocity to velocity [m/s]</param>
        /// <param name="velocity1">previous velocity to velocity0 [m/s]</param>
        /// <param name="dt">delta time step [s]</param>
        /// <returns>limiting factor (1.0 if none)</returns>
        public float LimitVelocityAndAcceleration(ref float velocity, float velocity0, float velocity1, float dt) {
            float tmp = velocity;
            LimitJerk(ref velocity, velocity0, velocity1, dt);
            LimitAcceleration(ref velocity, velocity0, dt);
            LimitVelocity(ref velocity);

            return tmp != 0.0f ? velocity / tmp : 1.0f;
        }

        public float LimitVelocity(ref float velocity) {
            float tmp = velocity;
            if (_hasVelocityLimits) {
                velocity = UnityEngine.Mathf.Clamp(velocity, _minVelocity, _maxVelocity);
            }
            return tmp != 0.0f ? velocity / tmp : 1.0f;
        }

        public float LimitAcceleration(ref float velocity, float velocity0, float dt) {
            float tmp = velocity;
            if (_hasAccelerationLimits) {
                float dv_min = _minAcceleration * dt;
                float dv_max = _maxAcceleration * dt;
                float dv = UnityEngine.Mathf.Clamp(velocity - velocity0, dv_min, dv_max);
                velocity = velocity0 + dv;
            }
            return tmp != 0.0f ? velocity / tmp : 1.0f;
        }

        public float LimitJerk(ref float velocity, float velocity0, float velocity1, float dt) {
            float tmp = velocity;
            if (_hasJerkLimits) {
                float dv = velocity - velocity0;
                float dv0 = velocity0 - velocity1;
                float dt2 = 2.0f * dt * dt;
                float da_min = _minJerk * dt2;
                float da_max = _maxJerk * dt2;
                float da = UnityEngine.Mathf.Clamp(dv - dv0, da_min, da_max);
                velocity = velocity0 + dv0 + da;
            }
            return tmp != 0.0f ? velocity / tmp : 1.0f;
        }
    }
}