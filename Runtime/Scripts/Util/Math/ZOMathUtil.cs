using System.Xml.Schema;
using System.Numerics;
using System;

namespace ZO.Math {
    public static class ZOMathUtil {
        public static double DegreesToRadians(double degrees) {
            double radians = (System.Math.PI / 180) * degrees;
            return (radians);
        }

        public static double RadiansToDegrees(double angle) {
            return angle * (180.0 / System.Math.PI);
        }

        public static bool isApproximatelyEqual(double a, double b, double tolerance = Double.Epsilon) {
            double diff = System.Math.Abs(a - b);
            if (diff <= tolerance)
                return true;

            if (diff < System.Math.Max(System.Math.Abs(a), System.Math.Abs(b)) * tolerance)
                return true;

            return false;
        }

        //supply tolerance that is meaningful in your context
        //for example, default tolerance may not work if you are comparing double with float
        public static bool isApproximatelyZero(double a, double tolerance = Double.Epsilon) {
            if (System.Math.Abs(a) <= tolerance)
                return true;
            return false;
        }

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

        public static float lowPassFilter(float currentValue, float prevValue, float alpha) {
            float result = (alpha * currentValue) + ((1.0f - alpha) * prevValue);
            return result;
        }

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
    }
}