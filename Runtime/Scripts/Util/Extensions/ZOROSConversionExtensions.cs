using UnityEngine;
using System.Collections.Generic;

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

        public static Quaternion RosRollPitchYawToQuaternion(this Vector3 rpy) {
            // Negate X and Z because we're going from Right to Left handed rotations. Y is handled because the axis itself is flipped
            rpy.x *= -1;
            rpy.z *= -1;
            rpy *= Mathf.Rad2Deg;

            // swap the angle values
            rpy = new Vector3(rpy.y, rpy.z, rpy.x);

            // Applying rotations in ZYX ordering, as indicated above
            Quaternion q = Quaternion.identity;

            q *= Quaternion.Euler(0, rpy.y, 0);
            q *= Quaternion.Euler(rpy.x, 0, 0);
            q *= Quaternion.Euler(0, 0, rpy.z);


            return q;
        }

        public static string ToXMLString(this Vector3 v) {
            return $"{v.x} {v.y} {v.z}";
        }

        /// <summary>
        /// Converts URDF Xml string vector "1 2 3" to Vector3
        /// </summary>
        /// <param name="s">URDF XML string in format "1.0 2.0 3.0" 3 numbers separated by spaces</param>
        /// <returns></returns>
        public static Vector3 FromURDFStringToVector3(this string s) {
            string[] splits = s.Split(' ');
            List<float> numbers = new List<float>();
            foreach (string snum in splits) {
                if (float.TryParse(snum, out float v)) {
                    numbers.Add(v);
                }
            }

            if (numbers.Count == 3) {
                return new Vector3(numbers[0], numbers[1], numbers[2]);
            }

            Debug.LogWarning($"Could not parse string: {s}");

            return Vector3.zero;
        }


    }

}
