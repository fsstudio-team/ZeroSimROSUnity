using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ZO.Util {

    
    public class ZOMagnetometer : MonoBehaviour {

        public ZO.Util.ZOEarthUtils.GeoPoint _geoPoint;
        public ZO.Math.ZOGaussianNoiseModel _noise = new ZO.Math.ZOGaussianNoiseModel {
            Mean = 0.0,
            StdDev = 0.01,
            BiasMean = 0,
            BiasStdDev = 0
        };

        private Vector3 _magneticField;
        private Vector3 _worldMagneticField;
        public Vector3 MagneticField {
            get { return _magneticField; }
        }

        // Start is called before the first frame update
        void Start() {
            _worldMagneticField = ZO.Util.ZOEarthUtils.GetMagField(_geoPoint) * 1e4f;
        }

        // Update is called once per frame
        void FixedUpdate() {
            _magneticField = transform.InverseTransformDirection(_worldMagneticField);
            _magneticField = _noise.Apply(_magneticField);
        }
    }
}