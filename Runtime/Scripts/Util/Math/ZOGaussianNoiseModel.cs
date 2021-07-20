using UnityEngine;
using System;

namespace ZO.Math {
    // See: https://bitbucket.org/jgoppert/gazebo/src/26c0e97eff2dca0c79a8601213d4e8826057f385/gazebo/sensors/GaussianNoiseModel.cc
    [Serializable]
    public class ZOGaussianNoiseModel {
        public double _mean = 0;
        public double Mean {
            get { return _mean; }
            set { _mean = value; }
        }
        public double _stdDev = 0;
        public double StdDev {
            get { return _stdDev; }
            set { _stdDev = value; }
        }
        public double _biasMean = 0;
        public double BiasMean {
            get { return _biasMean; }
            set {
                _biasMean = value;
                _needToCalculateBias = System.Math.Abs(value) > 0 ? true : false;
            }
        }

        public double _biasStdDev = 0;
        public double BiasStdDev {
            get { return _biasStdDev; }
            set {
                _biasStdDev = value;
                _needToCalculateBias = System.Math.Abs(value) > 0 ? true : false;
            }
        }

        private System.Random _random = new System.Random();
        private bool _needToCalculateBias = false;
        private double _bias = 0.0;

        public double Apply(double v) {
            if (_needToCalculateBias == true) {  // initialize the bias
                _bias = ZORandom.NextGaussian(_random, _biasMean, _biasStdDev);
                if (_random.NextDouble() < 0.5) {
                    _bias = -_bias;
                }
            }
            double whiteNoise = ZO.Math.ZORandom.NextGaussian(_random, _mean, _stdDev);
            double output = v + _bias + whiteNoise;
            return output;
        }

        public float Apply(float v) {
            return (float)Apply((double)v);
        }

        public Vector3 Apply(Vector3 v) {
            Vector3 output = new Vector3(Apply(v.x), Apply(v.y), Apply(v.z));
            return output;
        }
    }
}