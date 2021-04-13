using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ZO.Util;

namespace ZO.Controllers {

    public class ZOQuadCopterController : ZOGameObjectBase {
        
        public enum ZOQuadCopterMotorConfiguration {
            XConfiguration,
            CrossConfiguration
        }

        public ZODroneMotorConfiguration _quadCopterConfiguration = ZOQuadCopterMotorConfiguration.XConfiguration;
        // Start is called before the first frame update
        void Start() {

        }

        // Update is called once per frame
        void Update() {

        }
    }
}