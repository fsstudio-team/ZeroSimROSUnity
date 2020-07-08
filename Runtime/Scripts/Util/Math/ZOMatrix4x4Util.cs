using UnityEngine;

namespace ZO.Math {
    public static class ZOMatrix4x4Util {
        public static Vector3 GetTranslation(this Matrix4x4 m) {
            var col = m.GetColumn(3);
            return new Vector3(col.x, col.y, col.z);
        }

        public static Quaternion GetRotation(this Matrix4x4 m) {
            // Adapted from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
            // Quaternion q = new Quaternion();
            // q.w = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2;
            // q.x = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2;
            // q.y = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2;
            // q.z = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2;
            // q.x *= Mathf.Sign(q.x * (m[2, 1] - m[1, 2]));
            // q.y *= Mathf.Sign(q.y * (m[0, 2] - m[2, 0]));
            // q.z *= Mathf.Sign(q.z * (m[1, 0] - m[0, 1]));
            // return q;

            Quaternion q = Quaternion.LookRotation(m.GetColumn(2), m.GetColumn(1));
            return q;
        }

        public static Vector3 GetScale(this Matrix4x4 m) {
            return new Vector3(m.GetColumn(0).magnitude, m.GetColumn(1).magnitude, m.GetColumn(2).magnitude);
        }
    }
}