using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using ZO.Util;

namespace ZO.Sensors {


    /// <summary>
    /// Altimeter sensor.
    /// 
    /// Notes:  Based from Hector Quadrotor. See: https://github.com/udacity/RoboND-QuadRotor-Unity-Simulator/blob/1e71321d95e416bafa5296a229f36404bddc0452/Assets/Scripts/ROS/Hector_Quadrotor/hector_uav_msgs/Altimeter.cs
    /// </summary>
    public class ZOAltimeter : ZOGameObjectBase {


        public const float STANDARD_PRESSURE = 1013.25f;

        public float _QNH = STANDARD_PRESSURE;

        public float QNH {
            get => _QNH;
        }

        public float _startAltitude = 0.0f;
        public float StartAltitude {
            get => _startAltitude;
        }

        [Header("Noise Model")]
        public ZO.Math.ZOGaussianNoiseModel _noiseModel = new Math.ZOGaussianNoiseModel {
            Mean = 0.0,
            StdDev = 2e-4,
            BiasMean = 75e-6,
            BiasStdDev = 8e-08
        };

        /// <summary>
        /// Called every frame.
        /// Parameters: ZOAltimeter:this, altitude, pressure, qnh
        /// </summary>
        /// <value></value>
        public Func<ZOAltimeter, float, float, float, Task> OnPublishDelegate { get; set; }

        public float AltitudeMeters {
            get; set;
        }

        public static float AltitudeFromPressure(float pressure, float qnh = ZOAltimeter.STANDARD_PRESSURE) {
            return (float)(288.15 / 0.0065 * (1.0 - System.Math.Pow(pressure / qnh, 1.0 / 5.255)));
        }

        public static float PressureFromAltitude(float altitude, float qnh = ZOAltimeter.STANDARD_PRESSURE) {
            return (float)(qnh * System.Math.Pow(1.0 - (0.0065 * altitude) / 288.15, 5.255));
        }

        protected override void ZOStart() {
            base.ZOStart();
            AltitudeMeters = 0.0f;
        }

        protected async override void ZOFixedUpdateHzSynchronized() {
            base.ZOFixedUpdateHzSynchronized();

            float altitude = StartAltitude + transform.position.y;
            AltitudeMeters = _noiseModel.Apply(altitude);

            if (OnPublishDelegate != null) {
                await OnPublishDelegate(this, AltitudeMeters, PressureFromAltitude(AltitudeMeters), QNH);
            }
        }

    }
}