using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.Util {
    public class ZOTurnOffSelfCollision : MonoBehaviour {
        private void Awake() {
            Collider[] colliders = GetComponentsInChildren<Collider>();
            for (int i = 0; i < colliders.Length; i++) {
                for (int j = i; j < colliders.Length; j++) {
                    UnityEngine.Physics.IgnoreCollision(colliders[i], colliders[j]);
                }
            }
        }
    }

}
