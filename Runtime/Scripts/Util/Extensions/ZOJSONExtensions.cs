using Newtonsoft.Json.Linq;
using Newtonsoft.Json;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.Util.Extensions {
    public static class ZOJSONExtensions {

        /// <summary>
        /// Get value of key or if does not exist returns supplied default value.
        /// NOTE:  will not work for "non-simple" values such as Vector3 or Quaternion.
        /// </summary>
        /// <param name="json">JObject that we are extending</param>
        /// <param name="key">JSON key</param>
        /// <param name="defaultValue">Default value if key does not exist.</param>
        /// <typeparam name="T"></typeparam>
        /// <returns>Value</returns>
        static public T ValueOrDefault<T>(this JObject json, string key, T defaultValue) {
            // make sure that we are not doing Vector3 or Quaternion
            UnityEngine.Assertions.Assert.AreNotEqual(typeof(T), typeof(Vector3));
            UnityEngine.Assertions.Assert.AreNotEqual(typeof(T), typeof(Quaternion));

            return json.ContainsKey(key) ? json[key].Value<T>() : defaultValue;
        }

        /// <summary>
        /// Get Vector3 value of key or supplied default value.
        /// </summary>
        /// <param name="json"></param>
        /// <param name="key"></param>
        /// <param name="defaultValue"></param>
        /// <returns></returns>
        static public Vector3 ToVector3OrDefault(this JObject json, string key, Vector3 defaultValue) {
            return json.ContainsKey(key) ? json[key].ToVector3() : defaultValue;
        }

        /// <summary>
        /// Get Vector2 value of key or supplied default value.
        /// </summary>
        /// <param name="json"></param>
        /// <param name="key"></param>
        /// <param name="defaultValue"></param>
        /// <returns></returns>
        static public Vector2 ToVector2OrDefault(this JObject json, string key, Vector2 defaultValue) {
            return json.ContainsKey(key) ? json[key].ToVector2() : defaultValue;
        }

        /// <summary>
        /// Get Quaternion value of key or supplied default value.
        /// </summary>
        /// <param name="json"></param>
        /// <param name="key"></param>
        /// <param name="defaultValue"></param>
        /// <returns></returns>
        static public Quaternion ToQuaternionOrDefault(this JObject json, string key, Quaternion defaultValue) {
            return json.ContainsKey(key) ? json[key].ToQuaternion() : defaultValue;
        }

        /// <summary>
        /// Get Color value of key or supplied default value.
        /// </summary>
        /// <param name="json"></param>
        /// <param name="key"></param>
        /// <param name="defaultValue"></param>
        /// <returns></returns>
        static public Color ToColorOrDefault(this JObject json, string key, Color defaultValue) {
            return json.ContainsKey(key) ? json[key].ToColor() : defaultValue;
        }

        /// <summary>
        /// Convert JSON token to Unity Vector3.
        /// </summary>
        /// <param name="json">JSON Token. Must be a JArray!</param>
        /// <returns></returns>
        static public Vector3 ToVector3(this JToken json) {
            List<float> v3list = json.ToObject<List<float>>();
            return new Vector3(v3list[0], v3list[1], v3list[2]);
        }

        /// <summary>
        /// Convert Unity Vector3 to JSON token.
        /// </summary>
        /// <param name="v">Unity Vector3</param>
        /// <returns>JSON token</returns>
        static public JToken ToJSON(this Vector3 v) {
            return new JArray(v.x, v.y, v.z);
        }

        /// <summary>
        /// Convert JSON token to Unity Quaternion.
        /// </summary>
        /// <param name="json">JSON Token. Must be a JArray!</param>
        /// <returns>Unity Quaternion</returns>
        static public Quaternion ToQuaternion(this JToken json) {
            List<float> qlist = json.ToObject<List<float>>();
            return new Quaternion(qlist[0], qlist[1], qlist[2], qlist[3]);
        }

        /// <summary>
        /// Convert Unity Quaternion to JSON token. 
        /// </summary>
        /// <param name="v">Unity Quaternion</param>
        /// <returns>JSON token</returns>
        static public JToken ToJSON(this Quaternion v) {
            return new JArray(v.x, v.y, v.z, v.w);
        }


        /// <summary>
        /// Convert JSON token to Unity Color. RGBA order.
        /// </summary>
        /// <param name="json">JSON Token. Must be a JArray! (R, G, B, A)</param>
        /// <returns>Unity Color</returns>
        static public Color ToColor(this JToken json) {
            List<float> qlist = json.ToObject<List<float>>();
            return new Color(qlist[0], qlist[1], qlist[2], qlist[3]);
        }

        /// <summary>
        /// Convert Unity Color to JSON token. 
        /// </summary>
        /// <param name="v">Unity Quaternion</param>
        /// <returns>JSON token</returns>
        static public JToken ToJSON(this Color v) {
            return new JArray(v.r, v.g, v.b, v.a);
        }

        /// <summary>
        /// Convert JSON token to Unity Vector2.
        /// </summary>
        /// <param name="json">JSON Token. Must be a JArray! (R, G, B, A)</param>
        /// <returns>Unity Color</returns>
        static public Vector2 ToVector2(this JToken json) {
            List<float> qlist = json.ToObject<List<float>>();
            return new Vector2(qlist[0], qlist[1]);
        }

        /// <summary>
        /// Convert Unity Vector2 to JSON token. 
        /// </summary>
        /// <param name="v">Unity Quaternion</param>
        /// <returns>JSON token</returns>
        static public JToken ToJSON(this Vector2 v) {
            return new JArray(v.x, v.y);
        }

    }
}