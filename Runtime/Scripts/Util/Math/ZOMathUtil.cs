using System.Xml.Schema;
using System.Numerics;
using System;

namespace ZO.Math {
    public static class ZOMathUtil {

        /// <summary>
        /// Convert double degrees to radians.
        /// </summary>
        /// <param name="degrees">Input angle in degrees.</param>
        /// <returns>Output angle in radians.</returns>
        public static double DegreesToRadians(double degrees) {
            double radians = (System.Math.PI / 180) * degrees;
            return (radians);
        }

        /// <summary>
        /// Converts double radians to degrees.
        /// </summary>
        /// <param name="angle">Input angle in radians.</param>
        /// <returns>Outpus angle in degrees.</returns>
        public static double RadiansToDegrees(double angle) {
            return angle * (180.0 / System.Math.PI);
        }


        /// <summary>
        /// Checks if two double values are approximately equal.
        /// </summary>
        /// <param name="a">First value.</param>
        /// <param name="b">Second value.</param>
        /// <param name="tolerance">The epsilon cut off.</param>
        /// <returns></returns>
        public static bool isApproximatelyEqual(double a, double b, double tolerance = Double.Epsilon) {
            double diff = System.Math.Abs(a - b);
            if (diff <= tolerance)
                return true;

            if (diff < System.Math.Max(System.Math.Abs(a), System.Math.Abs(b)) * tolerance)
                return true;

            return false;
        }


        /// <summary>
        /// Checks if a value is approximately zero given an epsilon.
        /// </summary>
        /// <param name="a">The value</param>
        /// <param name="tolerance">The epsilon cut off.</param>
        /// <returns>True if zero otherwise false.</returns>
        public static bool isApproximatelyZero(double a, double tolerance = Double.Epsilon) {
            if (System.Math.Abs(a) <= tolerance)
                return true;
            return false;
        }

        /// <summary>
        /// Convert 2d cartesian coordinates to polar coordinates.
        /// </summary>
        /// <param name="point">2D point</param>
        /// <returns>2D polar coordinates.</returns>
        public static UnityEngine.Vector2 cartesianToPolar(UnityEngine.Vector2 point) {
            UnityEngine.Vector2 polar = new UnityEngine.Vector2();

            polar.x = UnityEngine.Mathf.Sqrt((point.x * point.x) + (point.y * point.y));

            // calc longitude 
            polar.y = UnityEngine.Mathf.Atan2(point.y, point.x);// * UnityEngine.Mathf.Rad2Deg;


            return polar;
        }


        public static Quaternion lowPassFilterQuaternion(Quaternion intermediateValue, Quaternion targetValue, float factor, bool init = false) {

            //intermediateValue needs to be initialized at the first usage.
            if (init) {
                intermediateValue = targetValue;
            }

            intermediateValue = Quaternion.Lerp(intermediateValue, targetValue, factor);
            return intermediateValue;
        }


        /// <summary>
        /// Simple low pass filter on `float`.
        /// </summary>
        /// <param name="currentValue">The current value.</param>
        /// <param name="prevValue">The previous value.</param>
        /// <param name="alpha">Weight 0..1 where greater value biases towards the current value.</param>
        /// <returns>Weighted value.</returns>
        public static float lowPassFilter(float currentValue, float prevValue, float alpha) {
            float result = (alpha * currentValue) + ((1.0f - alpha) * prevValue);
            return result;
        }



        /// <summary>
        /// Simple low pass filter on Vector3
        /// </summary>
        /// <param name="currentValue">The current value.</param>
        /// <param name="prevValue">The previous value.</param>
        /// <param name="alpha">Weight 0..1 where greater value biases towards the current value.</param>
        /// <returns>Weighted value.</returns>
        public static UnityEngine.Vector3 lowPassFilter(UnityEngine.Vector3 currentValue, UnityEngine.Vector3 prevValue, float alpha) {
            UnityEngine.Vector3 result = new UnityEngine.Vector3(
                                            lowPassFilter(currentValue.x, prevValue.x, alpha),
                                            lowPassFilter(currentValue.y, prevValue.y, alpha),
                                            lowPassFilter(currentValue.z, prevValue.z, alpha));
            return result;
        }

        public static UnityEngine.Vector3 lowPassFilter(UnityEngine.Vector3 targetValue, ref UnityEngine.Vector3 intermediateValueBuf, float factor, bool init) {

            UnityEngine.Vector3 intermediateValue = new UnityEngine.Vector3();

            //intermediateValue needs to be initialized at the first usage.
            if (init) {
                intermediateValueBuf = targetValue;
            }

            intermediateValue.x = (targetValue.x * factor) + (intermediateValueBuf.x * (1.0f - factor));
            intermediateValue.y = (targetValue.y * factor) + (intermediateValueBuf.y * (1.0f - factor));
            intermediateValue.z = (targetValue.z * factor) + (intermediateValueBuf.z * (1.0f - factor));

            intermediateValueBuf = intermediateValue;

            return intermediateValue;
        }


        /// <summary>
        /// Get the orthonormal basis given a start vector.
        /// </summary>
        /// <param name="normal">The start vector.</param>
        /// <param name="orthonormal1">The first ortho normal vector to the start vector.</param>
        /// <param name="orthonormal2">The second ortho normal vector to the start vector and the first ortho vector.</param>
        public static void FindOrthonormals(UnityEngine.Vector3 normal, out UnityEngine.Vector3 orthonormal1, out UnityEngine.Vector3 orthonormal2) {
            UnityEngine.Quaternion orthoX = UnityEngine.Quaternion.Euler(90, 0, 0);
            UnityEngine.Vector3 w = orthoX * normal;
            float dot = UnityEngine.Vector3.Dot(normal, w);
            if (UnityEngine.Mathf.Abs(dot) > 0.6) {
                UnityEngine.Quaternion orthoY = UnityEngine.Quaternion.Euler(0, 90, 0);
                w = orthoY * normal;
            }
            w.Normalize();

            orthonormal1 = UnityEngine.Vector3.Cross(normal, w);
            orthonormal1.Normalize();
            orthonormal2 = UnityEngine.Vector3.Cross(normal, orthonormal1);
            orthonormal2.Normalize();
        }

        /// <summary>
        /// Calculate the quaternion twist about an axis.
        /// <see>https://stackoverflow.com/questions/3684269/component-of-a-quaternion-rotation-around-an-axis</see>
        /// </summary>
        /// <param name="q">The quaternion.</param>
        /// <param name="axis">The axis.</param>
        /// <returns>The twist in radians.</returns>
        public static float FindQuaternionTwist(UnityEngine.Quaternion q, UnityEngine.Vector3 axis) {
            axis.Normalize();

            // Get the plane the axis is a normal of
            UnityEngine.Vector3 orthonormal1, orthonormal2;
            FindOrthonormals(axis, out orthonormal1, out orthonormal2);

            UnityEngine.Vector3 transformed = q * orthonormal1;

            // Project transformed vector onto plane
            UnityEngine.Vector3 flattened = transformed - (UnityEngine.Vector3.Dot(transformed, axis) * axis);
            flattened.Normalize();

            // Get angle between original vector and projected transform to get angle around normal
            float a = UnityEngine.Mathf.Acos(UnityEngine.Vector3.Dot(orthonormal1, flattened));

            return a;
        }
    }
}