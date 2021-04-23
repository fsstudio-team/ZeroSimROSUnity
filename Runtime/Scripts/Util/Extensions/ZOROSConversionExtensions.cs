using UnityEngine;

namespace ZO.Util.Extensions {
    public static class ZOROSConversionExtensions {
        public static Vector3 Ros2Unity(this Vector3 v) {
            return new Vector3(-v.y, v.z, v.x);
        }

        public static Vector3 Unity2Ros(this Vector3 v) {
            return new Vector3(v.z, -v.x, v.y);
        }

        public static Vector3 Ros2UnityScale(this Vector3 v) {
            return new Vector3(v.y, v.z, v.x);
        }

        public static Vector3 Unity2RosScale(this Vector3 v) {
            return new Vector3(v.z, v.x, v.y);
        }

        public static Quaternion Ros2Unity(this Quaternion q) {
            return new Quaternion(q.y, -q.z, -q.x, q.w);
        }

        public static Quaternion Unity2Ros(this Quaternion q) {
            return new Quaternion(-q.z, q.x, -q.y, q.w);
        }

        public static Vector3 Unity2RosRollPitchYaw(this Quaternion q) {
            return new Vector3(-q.eulerAngles.z * Mathf.Deg2Rad,
                            q.eulerAngles.x * Mathf.Deg2Rad,
                            -q.eulerAngles.y * Mathf.Deg2Rad);
        }

        public static string ToXMLString(this Vector3 v) {
            return $"{v.x} {v.y} {v.z}";
        }


    }

}
