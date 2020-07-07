using System;
using UnityEngine;

namespace ZO.Util {
    // See: https://github.com/microsoft/AirSim/blob/123bbd6d6416bbe98e775101ba8ebcb5ccbf8501/AirLib/include/common/EarthUtils.hpp
    public static class ZOEarthUtils {

        [Serializable]
        public struct GeoPoint {
            public double latitude;
            public double longitude;
            public float altitude;


            public GeoPoint(double latitude_val, double longitude_val, float altitude_val) {
                latitude = latitude_val;
                longitude = longitude_val;
                altitude = altitude_val;
            }

            public void set(double latitude_val, double longitude_val, float altitude_val) {
                latitude = latitude_val;
                longitude = longitude_val;
                altitude = altitude_val;
            }

        };

        public struct HomeGeoPoint {
            public GeoPoint home_geo_point;
            public double lat_rad, lon_rad;
            public double cos_lat, sin_lat;

            public HomeGeoPoint(GeoPoint home_geo_point_val) {
                home_geo_point = home_geo_point_val;
                lat_rad = ZO.Math.ZOMathUtil.DegreesToRadians(home_geo_point.latitude);
                lon_rad = ZO.Math.ZOMathUtil.DegreesToRadians(home_geo_point.longitude);
                cos_lat = System.Math.Cos(lat_rad);
                sin_lat = System.Math.Sin(lat_rad);
            }
            public void initialize(GeoPoint home_geo_point_val) {
                home_geo_point = home_geo_point_val;
                lat_rad = ZO.Math.ZOMathUtil.DegreesToRadians(home_geo_point.latitude);
                lon_rad = ZO.Math.ZOMathUtil.DegreesToRadians(home_geo_point.longitude);
                cos_lat = System.Math.Cos(lat_rad);
                sin_lat = System.Math.Sin(lat_rad);
            }
        };

        //ref: https://www.ngdc.noaa.gov/geomag/GeomagneticPoles.shtml
        public static double MagPoleLat = ZO.Math.ZOMathUtil.DegreesToRadians(80.31);
        public static double MagPoleLon = ZO.Math.ZOMathUtil.DegreesToRadians(-72.62f);
        public static double MeanMagField = 3.12E-5; //Tesla, https://en.wikipedia.org/wiki/Dipole_model_of_the_Earth's_magnetic_field
        public static float SeaLevelPressure = 101325.0f; //Pascal
        public static float SeaLevelAirDensity = 1.225f; //kg/m^3
        public static float Gravity = 9.80665f;    //m/s^2
        public static float Radius = EARTH_RADIUS; //m
        public static float SpeedOfLight = 299792458.0f; //m
        public static float Obliquity = (float)ZO.Math.ZOMathUtil.DegreesToRadians(23.4397f);
        public static double Perihelion = ZO.Math.ZOMathUtil.DegreesToRadians(102.9372); // perihelion of the Earth
        public static double DistanceFromSun = 149597870700.0; // meters

        public static float EARTH_RADIUS = 6378137.0f;

        /** set this always to the sampling in degrees for the table below */
        static int MAG_SAMPLING_RES = 10;
        static int MAG_SAMPLING_MIN_LAT = -60;
        static int MAG_SAMPLING_MAX_LAT = 60;
        static int MAG_SAMPLING_MIN_LON = -180;
        static int MAG_SAMPLING_MAX_LON = 180;

        static int[,] DECLINATION_TABLE = new int[,]
        {
            { 46, 45, 44, 42, 41, 40, 38, 36, 33, 28, 23, 16, 10, 4, -1, -5, -9, -14, -19, -26, -33, -40, -48, -55, -61, -66, -71, -74, -75, -72, -61, -25, 22, 40, 45, 47, 46 },
            { 30, 30, 30, 30, 29, 29, 29, 29, 27, 24, 18, 11, 3, -3, -9, -12, -15, -17, -21, -26, -32, -39, -45, -51, -55, -57, -56, -53, -44, -31, -14, 0, 13, 21, 26, 29, 30 },
            { 21, 22, 22, 22, 22, 22, 22, 22, 21, 18, 13, 5, -3, -11, -17, -20, -21, -22, -23, -25, -29, -35, -40, -44, -45, -44, -40, -32, -22, -12, -3, 3, 9, 14, 18, 20, 21 },
            { 16, 17, 17, 17, 17, 17, 16, 16, 16, 13, 8, 0, -9, -16, -21, -24, -25, -25, -23, -20, -21, -24, -28, -31, -31, -29, -24, -17, -9, -3, 0, 4, 7, 10, 13, 15, 16 },
            { 12, 13, 13, 13, 13, 13, 12, 12, 11, 9, 3, -4, -12, -19, -23, -24, -24, -22, -17, -12, -9, -10, -13, -17, -18, -16, -13, -8, -3, 0, 1, 3, 6, 8, 10, 12, 12 },
            { 10, 10, 10, 10, 10, 10, 10, 9, 9, 6, 0, -6, -14, -20, -22, -22, -19, -15, -10, -6, -2, -2, -4, -7, -8, -8, -7, -4, 0, 1, 1, 2, 4, 6, 8, 10, 10 },
            { 9, 9, 9, 9, 9, 9, 8, 8, 7, 4, -1, -8, -15, -19, -20, -18, -14, -9, -5, -2, 0, 1, 0, -2, -3, -4, -3, -2, 0, 0, 0, 1, 3, 5, 7, 8, 9 },
            { 8, 8, 8, 9, 9, 9, 8, 8, 6, 2, -3, -9, -15, -18, -17, -14, -10, -6, -2, 0, 1, 2, 2, 0, -1, -1, -2, -1, 0, 0, 0, 0, 1, 3, 5, 7, 8 },
            { 8, 9, 9, 10, 10, 10, 10, 8, 5, 0, -5, -11, -15, -16, -15, -12, -8, -4, -1, 0, 2, 3, 2, 1, 0, 0, 0, 0, 0, -1, -2, -2, -1, 0, 3, 6, 8 },
            { 6, 9, 10, 11, 12, 12, 11, 9, 5, 0, -7, -12, -15, -15, -13, -10, -7, -3, 0, 1, 2, 3, 3, 3, 2, 1, 0, 0, -1, -3, -4, -5, -5, -2, 0, 3, 6 },
            { 5, 8, 11, 13, 15, 15, 14, 11, 5, -1, -9, -14, -17, -16, -14, -11, -7, -3, 0, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -4, -7, -8, -8, -6, -2, 1, 5 },
            { 4, 8, 12, 15, 17, 18, 16, 12, 5, -3, -12, -18, -20, -19, -16, -13, -8, -4, -1, 1, 4, 6, 8, 9, 9, 9, 7, 3, -1, -6, -10, -12, -11, -9, -5, 0, 4 },
            { 3, 9, 14, 17, 20, 21, 19, 14, 4, -8, -19, -25, -26, -25, -21, -17, -12, -7, -2, 1, 5, 9, 13, 15, 16, 16, 13, 7, 0, -7, -12, -15, -14, -11, -6, -1, 3 },
        };


        /* magnetic field */
        static float get_mag_lookup_table_val(int lat_index, int lon_index) {
            return (float)(DECLINATION_TABLE[lat_index, lon_index]);
        }
        //return declination in degrees
        //Ref: https://github.com/PX4/ecl/blob/master/EKF/geo.cpp
        public static float GetMagDeclination(float latitude, float longitude) {
            /*
            * If the values exceed valid ranges, return zero as default
            * as we have no way of knowing what the closest real value
            * would be.
            */
            if (latitude < -90.0f || latitude > 90.0f ||
                longitude < -180.0f || longitude > 180.0f) {
                throw new ArgumentOutOfRangeException(String.Format("invalid latitude ({0}) or longitude ({1})", latitude, longitude));
            }

            /* round down to nearest sampling resolution */
            int min_lat = (int)(latitude / MAG_SAMPLING_RES) * MAG_SAMPLING_RES;
            int min_lon = (int)(longitude / MAG_SAMPLING_RES) * MAG_SAMPLING_RES;

            /* for the rare case of hitting the bounds exactly
            * the rounding logic wouldn't fit, so enforce it.
            */

            /* limit to table bounds - required for maxima even when table spans full globe range */
            if (latitude <= MAG_SAMPLING_MIN_LAT) {
                min_lat = MAG_SAMPLING_MIN_LAT;
            }

            if (latitude >= MAG_SAMPLING_MAX_LAT) {
                min_lat = (int)(latitude / MAG_SAMPLING_RES) * MAG_SAMPLING_RES - MAG_SAMPLING_RES;
            }

            if (longitude <= MAG_SAMPLING_MIN_LON) {
                min_lon = MAG_SAMPLING_MIN_LON;
            }

            if (longitude >= MAG_SAMPLING_MAX_LON) {
                min_lon = (int)(longitude / MAG_SAMPLING_RES) * MAG_SAMPLING_RES - MAG_SAMPLING_RES;
            }

            /* find index of nearest low sampling point */
            int min_lat_index = (-(MAG_SAMPLING_MIN_LAT) + min_lat) / MAG_SAMPLING_RES;
            int min_lon_index = (-(MAG_SAMPLING_MIN_LON) + min_lon) / MAG_SAMPLING_RES;

            float declination_sw = get_mag_lookup_table_val(min_lat_index, min_lon_index);
            float declination_se = get_mag_lookup_table_val(min_lat_index, min_lon_index + 1);
            float declination_ne = get_mag_lookup_table_val(min_lat_index + 1, min_lon_index + 1);
            float declination_nw = get_mag_lookup_table_val(min_lat_index + 1, min_lon_index);

            /* perform bilinear interpolation on the four grid corners */

            float declination_min = ((longitude - min_lon) / MAG_SAMPLING_RES) * (declination_se - declination_sw) + declination_sw;
            float declination_max = ((longitude - min_lon) / MAG_SAMPLING_RES) * (declination_ne - declination_nw) + declination_nw;

            return ((latitude - min_lat) / MAG_SAMPLING_RES) * (declination_max - declination_min) + declination_min;
        }

        //geopot_height = Earth_radius * altitude / (Earth_radius + altitude) /// all in kilometers
        //temperature is in Kelvin = 273.15 + celcius
        public static float GetStandardTemperature(float geopot_height) {
            //standard atmospheric pressure
            //Below 51km: Practical Meteorology by Roland Stull, pg 12
            //Above 51km: http://www.braeunig.us/space/atmmodel.htm
            if (geopot_height <= 11)          //troposphere
                return 288.15f - (6.5f * geopot_height);
            else if (geopot_height <= 20)     //Staroshere starts
                return 216.65f;
            else if (geopot_height <= 32)
                return 196.65f + geopot_height;
            else if (geopot_height <= 47)
                return 228.65f + 2.8f * (geopot_height - 32);
            else if (geopot_height <= 51)     //Mesosphere starts
                return 270.65f;
            else if (geopot_height <= 71)
                return 270.65f - 2.8f * (geopot_height - 51);
            else if (geopot_height <= 84.85f)
                return 214.65f - 2 * (geopot_height - 71);
            else return 3;
            //Thermospehere has high kinetic temperature (500c to 2000c) but temperature
            //as measured by thermometer would be very low because of almost vacuum
            //throw std::out_of_range("geopot_height must be less than 85km. Space domain is not supported yet!");
        }

        public static float GetGeopotential(float altitude_km) {
            float radius_km = EARTH_RADIUS / 1000.0f;
            return radius_km * altitude_km / (radius_km + altitude_km);
        }

        public static float getStandardPressure(float altitude /* meters */)    //return Pa
        {
            float geopot_height = GetGeopotential(altitude / 1000.0f);

            float t = GetStandardTemperature(geopot_height);

            return GetStandardPressure(geopot_height, t);
        }

        public static float GetStandardPressure(float geopot_height, float std_temperature)    //return Pa
        {
            //Below 51km: Practical Meteorology by Roland Stull, pg 12
            //Above 51km: http://www.braeunig.us/space/atmmodel.htm
            //Validation data: https://www.avs.org/AVS/files/c7/c7edaedb-95b2-438f-adfb-36de54f87b9e.pdf

            //TODO: handle -ve altitude better (shouldn't grow indefinitely!)

            if (geopot_height <= 11)
                //at alt 0, return sea level pressure
                return SeaLevelPressure * (float)System.Math.Pow(288.15f / std_temperature, -5.255877f);
            else if (geopot_height <= 20)
                return 22632.06f * (float)System.Math.Exp(-0.1577f * (geopot_height - 11));
            else if (geopot_height <= 32)
                return 5474.889f * (float)System.Math.Pow(216.65f / std_temperature, 34.16319f);
            else if (geopot_height <= 47)
                return 868.0187f * (float)System.Math.Pow(228.65f / std_temperature, 12.2011f);
            else if (geopot_height <= 51)
                return 110.9063f * (float)System.Math.Exp(-0.1262f * (geopot_height - 47));
            else if (geopot_height <= 71)
                return 66.93887f * (float)System.Math.Pow(270.65f / std_temperature, -12.2011f);
            else if (geopot_height <= 84.85f)
                return 3.956420f * (float)System.Math.Pow(214.65f / std_temperature, -17.0816f);
            else return 1E-3f;
            //throw std::out_of_range("altitude must be less than 86km. Space domain is not supported yet!");    
        }

        public static float GetAirDensity(float std_pressure, float std_temperature)  //kg / m^3
        {
            //http://www.braeunig.us/space/atmmodel.htm
            return std_pressure / 287.053f / std_temperature;
        }

        public static float GetAirDensity(float altitude)  //kg / m^3
        {
            float geo_pot = GetGeopotential(altitude / 1000.0f);
            float std_temperature = GetStandardTemperature(geo_pot);
            float std_pressure = GetStandardPressure(geo_pot, std_temperature);
            return GetAirDensity(std_pressure, std_temperature);
        }

        public static float GetSpeedofSound(float altitude)  // m/s
        {
            //http://www.braeunig.us/space/atmmodel.htm
            return (float)System.Math.Sqrt(1.400f * 287.053f * GetStandardTemperature(GetGeopotential(altitude)));
        }

        public static float GetGravity(float altitude) {
            //derivation: http://www.citycollegiate.com/gravitation_XId.htm
            if (altitude < 10000 && altitude > -10000)   //up to 10 km, difference is too small
                return ZOEarthUtils.Gravity;
            if (altitude < 100000 && altitude > -100000)   //use first exproximation using binomial expansion
                return ZOEarthUtils.Gravity * (1 - 2 * altitude / EARTH_RADIUS);
            else {
                float factor = 1 + altitude / EARTH_RADIUS;
                return ZOEarthUtils.Gravity / factor / factor;
            }
        }

        public static Vector3 GetMagField(GeoPoint geo_point)  //return Tesla
        {
            double declination = 0, inclination = 0;
            return GetMagField(geo_point, declination, inclination);
        }

        public static Vector3 GetMagField(GeoPoint geo_point, double declination = 0, double inclination = 0)  //return Tesla
        {
            /*
            We calculate magnetic field using simple dipol model of Earth, i.e., assume
            earth as perfect dipole sphere and ignoring all but first order terms.
            This obviously is inaccurate because of huge amount of irregularities, magnetic pole that is
            constantly moving, shape of Earth, higher order terms, dipole that is not perfectly aligned etc.
            For simulation we are not looking for actual values of magnetic field but rather if field changes
            correctly as vehicle moves in any direction and if field component signs are correct. For this purpose, simple
            diapole model is good enough. Keep in mind that actual field values may differ by as much as 10X in either direction
            although for many tests differences seems to be within 3X or sometime even to first decimal digit. Again what matters is
            how field changes wrt to movement as opposed to actual field values. To get better field strength one should use latest 
            World Magnetic Model like WMM2015 from NOAA. However these recent model is fairly complex and very expensive to calculate. 
            Other possibilities: 
                - WMM2010 mocel, expensive to compute: http://williams.best.vwh.net/magvar/magfield.c
                - Android's mag field calculation (still uses WMM2010 and fails at North Pole): https://goo.gl/1CZB9x
            Performance:
                This function takes about 1 microsecond on Lenovo P50 laptop (Intel Xeon E3-1505M v5 CPU)
                Basic trignometry functions runs at 30ns.
            Accuracy:
                Two points separated by sqrt(2 km)
                Dipole Model:   2.50394e-05     3.40771e-06     3.6567e-05  (dec: 7.7500, inc: 55.3530)
                WMM2015 Model:  1.8350e-05		5.201e-06		5.0158e-05  (dec: 15.8248, inc: 69.1805)
                geo:            47.637  -122.147    622
                Dipole Model:   2.5047e-05      3.41024e-06     3.65953e-05 (dec: 7.7536, inc: 55.36532)
                WMM2015 Model:  1.8353e-05		5.203e-06		5.0191e-05  (dec: 15.8278, inc: 69.1897)
                geo:            47.646  -122.134    -378
            */

            //ref: The Earth's Magnetism: An Introduction for Geologists, Roberto Lanza, Antonio Meloni
            //Sec 1.2.5, pg 27-30 https://goo.gl/bRm7wt
            //some theory at http://www.tulane.edu/~sanelson/eens634/Hmwk6MagneticField.pdf

            double lat = ZO.Math.ZOMathUtil.DegreesToRadians(geo_point.latitude); //geographic colatitude
            double lon = ZO.Math.ZOMathUtil.DegreesToRadians(geo_point.longitude);
            double altitude = geo_point.altitude + EARTH_RADIUS;

            //cache value
            double sin_MagPoleLat = System.Math.Sin(MagPoleLat);
            double cos_MagPoleLat = System.Math.Cos(MagPoleLat);
            double cos_lat = System.Math.Cos(lat);
            double sin_lat = System.Math.Sin(lat);

            //find magnetic colatitude
            double mag_clat = System.Math.Acos(cos_lat * cos_MagPoleLat +
                sin_lat * sin_MagPoleLat * System.Math.Cos(lon - MagPoleLon));

            //calculation of magnetic longitude is not needed but just in case if someone wants it
            //double mag_lon = asin( 
            //    (sin(lon - MagPoleLon) * sin(lat)) /
            //    sin(mag_clat));

            //field strength only depends on magnetic colatitude
            //https://en.wikipedia.org/wiki/Dipole_model_of_the_Earth's_magnetic_field
            double cos_mag_clat = System.Math.Cos(mag_clat);
            double field_mag = MeanMagField * System.Math.Pow(EARTH_RADIUS / altitude, 3) *
                System.Math.Sqrt
                (1 + 3 * cos_mag_clat * cos_mag_clat);

            //find inclination and declination
            //equation of declination in above referenced book is only partial
            //full equation is (4a) at http://www.tulane.edu/~sanelson/eens634/Hmwk6MagneticField.pdf
            double lat_test = sin_MagPoleLat * sin_lat;
            double dec_factor = cos_MagPoleLat / System.Math.Sin(mag_clat);
            if (cos_mag_clat > lat_test)
                declination = System.Math.Asin(System.Math.Sin(lon - MagPoleLon) * dec_factor);
            else
                declination = System.Math.Asin(System.Math.Cos(lon - MagPoleLon) * dec_factor);
            inclination = System.Math.Atan(2.0 / System.Math.Tan(mag_clat)); //do not use atan2 here

            //transform magnetic field vector to geographical coordinates
            //ref: http://www.geo.mtu.edu/~jdiehl/magnotes.html
            double field_xy = field_mag * System.Math.Cos(inclination);
            return new Vector3(
                (float)(field_xy * System.Math.Cos(declination)),
                (float)(field_xy * System.Math.Sin(declination)),
                (float)(field_mag * System.Math.Sin(inclination))
            );
        }

        public static GeoPoint NedToGeodetic(Vector3 v, HomeGeoPoint home_geo_point) {
            double x_rad = v.x / EARTH_RADIUS;
            double y_rad = v.y / EARTH_RADIUS;
            double c = System.Math.Sqrt(x_rad * x_rad + y_rad * y_rad);
            double sin_c = System.Math.Sin(c), cos_c = System.Math.Cos(c);
            double lat_rad, lon_rad;
            if (ZO.Math.ZOMathUtil.isApproximatelyZero(c) == false) { //avoids large changes?
                lat_rad = System.Math.Asin(cos_c * home_geo_point.sin_lat + (x_rad * sin_c * home_geo_point.cos_lat) / c);
                lon_rad = (home_geo_point.lon_rad +
                    System.Math.Atan2(y_rad * sin_c, c * home_geo_point.cos_lat * cos_c - x_rad * home_geo_point.sin_lat * sin_c));

                return new GeoPoint(ZO.Math.ZOMathUtil.RadiansToDegrees(lat_rad), ZO.Math.ZOMathUtil.RadiansToDegrees(lon_rad),
                    home_geo_point.home_geo_point.altitude - v.z);
            } else
                return new GeoPoint(home_geo_point.home_geo_point.latitude, home_geo_point.home_geo_point.longitude, home_geo_point.home_geo_point.altitude - v.z);
        }

        //below are approximate versions and would produce errors of more than 10m for points farther than 1km
        //for more accurate versions, please use the version in EarthUtils::nedToGeodetic
        public static Vector3 GeodeticToNedFast(GeoPoint geo, GeoPoint home) {
            double d_lat = geo.latitude - home.latitude;
            double d_lon = geo.longitude - home.longitude;
            float d_alt = (float)(home.altitude - geo.altitude);
            float x = (float)(ZO.Math.ZOMathUtil.DegreesToRadians(d_lat) * EARTH_RADIUS);
            float y = (float)(ZO.Math.ZOMathUtil.DegreesToRadians(d_lon) * EARTH_RADIUS * System.Math.Cos(ZO.Math.ZOMathUtil.DegreesToRadians(geo.latitude)));
            return new Vector3(x, y, d_alt);
        }
        public static GeoPoint NedToGeodeticFast(Vector3 local, GeoPoint home) {
            GeoPoint r;
            double d_lat = local.x / EARTH_RADIUS;
            double d_lon = local.y / (EARTH_RADIUS * System.Math.Cos(ZO.Math.ZOMathUtil.DegreesToRadians(home.latitude)));
            r.latitude = home.latitude + ZO.Math.ZOMathUtil.RadiansToDegrees(d_lat);
            r.longitude = home.longitude + ZO.Math.ZOMathUtil.RadiansToDegrees(d_lon);
            r.altitude = home.altitude - local.z;

            return r;
        }


    }
}
