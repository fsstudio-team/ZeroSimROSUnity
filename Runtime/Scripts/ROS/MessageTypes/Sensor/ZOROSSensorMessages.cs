using System;

using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.Geometry;

namespace ZO.ROS.MessageTypes.Sensor {

    /// <summary>
    ///  This message contains an uncompressed image
    ///  (0, 0) is at top-left corner of image
    /// </summary>
    public class ImageMessage : ZOROSMessageInterface {


        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return ImageMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "sensor_msgs/Image";


        public HeaderMessage header { get; set; }
        //  Header timestamp should be acquisition time of image
        //  Header frame_id should be optical frame of camera
        //  origin of frame should be optical center of camera
        //  +x should point to the right in the image
        //  +y should point down in the image
        //  +z should point into to plane of the image
        //  If the frame_id here and the frame_id of the CameraInfo
        //  message associated with the image conflict
        //  the behavior is undefined
        public uint height { get; set; }
        //  image height, that is, number of rows
        public uint width { get; set; }
        //  image width, that is, number of columns
        //  The legal values for encoding are in file src/image_encodings.cpp
        //  If you want to standardize a new string format, join
        //  ros-users@lists.sourceforge.net and send an email proposing a new encoding.
        public string encoding { get; set; }
        //  Encoding of pixels -- channel meaning, ordering, size
        //  taken from the list of strings in include/sensor_msgs/image_encodings.h
        public byte is_bigendian { get; set; }
        //  is this data bigendian?
        public uint step { get; set; }
        //  Full row length in bytes
        public byte[] data { get; set; }
        //  actual matrix data, size is (step * rows)

        public ImageMessage() {
            this.header = new HeaderMessage();
            this.height = 0;
            this.width = 0;
            this.encoding = "";
            this.is_bigendian = 0;
            this.step = 0;
            this.data = new byte[0];
        }

        public ImageMessage(HeaderMessage header, uint height, uint width, string encoding, byte is_bigendian, uint step, byte[] data) {
            this.header = header;
            this.height = height;
            this.width = width;
            this.encoding = encoding;
            this.is_bigendian = is_bigendian;
            this.step = step;
            this.data = data;
        }

        public void Update() {
            this.header.Update();
        }
    }


    /// <summary>
    /// Single scan from a planar laser range-finder
    ///
    /// If you have another ranging device with different behavior (e.g. a sonar
    /// array), please find or create a different message, since applications
    /// will make fairly laser-specific assumptions about this data
    /// <see>http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html</see>
    /// </summary>
    public class LaserScanMessage : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return LaserScanMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "sensor_msgs/LaserScan";


        public HeaderMessage header { get; set; }

        //  in frame frame_id, angles are measured around 
        //  the positive Z axis (counterclockwise, if Z is up)
        //  with zero angle being forward along the x axis
        public float angle_min { get; set; }
        //  start angle of the scan [rad]
        public float angle_max { get; set; }
        //  end angle of the scan [rad]
        public float angle_increment { get; set; }
        //  angular distance between measurements [rad]
        public float time_increment { get; set; }
        //  time between measurements [seconds] - if your scanner
        //  is moving, this will be used in interpolating position
        //  of 3d points
        public float scan_time { get; set; }
        //  time between scans [seconds]
        public float range_min { get; set; }
        //  minimum range value [m]
        public float range_max { get; set; }
        //  maximum range value [m]
        public float[] ranges { get; set; }
        //  range data [m] (Note: values < range_min or > range_max should be discarded)
        public float[] intensities { get; set; }
        //  intensity data [device-specific units].  If your
        //  device does not provide intensities, please leave
        //  the array empty.

        public LaserScanMessage() {
            this.header = new HeaderMessage();
            this.angle_min = 0.0f;
            this.angle_max = 0.0f;
            this.angle_increment = 0.0f;
            this.time_increment = 0.0f;
            this.scan_time = 0.0f;
            this.range_min = 0.0f;
            this.range_max = 0.0f;
            this.ranges = new float[0];
            this.intensities = new float[0];
        }

        public LaserScanMessage(HeaderMessage header, float angle_min, float angle_max, float angle_increment, float time_increment, float scan_time, float range_min, float range_max, float[] ranges, float[] intensities) {
            this.header = header;
            this.angle_min = angle_min;
            this.angle_max = angle_max;
            this.angle_increment = angle_increment;
            this.time_increment = time_increment;
            this.scan_time = scan_time;
            this.range_min = range_min;
            this.range_max = range_max;
            this.ranges = ranges;
            this.intensities = intensities;
        }
    }

    /// <summary>
    /// This is a message that holds data to describe the state of a set of torque controlled joints. 
    /// 
    ///  The state of each joint (revolute or prismatic) is defined by:
    ///   * the position of the joint (rad or m),
    ///   * the velocity of the joint (rad/s or m/s) and 
    ///   * the effort that is applied in the joint (Nm or N).
    /// 
    ///  Each joint is uniquely identified by its name
    ///  The header specifies the time at which the joint states were recorded. All the joint states
    ///  in one message have to be recorded at the same time.
    /// 
    ///  This message consists of a multiple arrays, one for each part of the joint state. 
    ///  The goal is to make each of the fields optional. When e.g. your joints have no
    ///  effort associated with them, you can leave the effort array empty. 
    /// 
    ///  All arrays in this message should have the same size, or be empty.
    ///  This is the only way to uniquely associate the joint name with the correct
    ///  states.
    /// </summary>
    public class JointStateMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return JointStateMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type { get { return "sensor_msgs/JointState"; } }

        public HeaderMessage header { get; set; }
        public string[] name { get; set; }
        public double[] position { get; set; }
        public double[] velocity { get; set; }
        public double[] effort { get; set; }

        public JointStateMessage() {
            this.header = new HeaderMessage();
            this.name = new string[0];
            this.position = new double[0];
            this.velocity = new double[0];
            this.effort = new double[0];
        }

        public JointStateMessage(HeaderMessage header, string[] name, double[] position, double[] velocity, double[] effort) {
            this.header = header;
            this.name = name;
            this.position = position;
            this.velocity = velocity;
            this.effort = effort;
        }

        public void Update() {
            this.header.Update();
        }
    }

    /// <summary>
    /// This is a message to hold data from an IMU (Inertial Measurement Unit)
    ///
    /// Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    ///
    /// If the covariance of the measurement is known, it should be filled in (if all you know is the 
    /// variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
    /// A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
    /// data a covariance will have to be assumed or gotten from some other source
    ///
    /// If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
    /// estimate), please set element 0 of the associated covariance matrix to -1
    /// If you are interpreting this message, please check for a value of -1 in the first element of each 
    /// covariance matrix, and disregard the associated estimate.
    /// </summary>
    public class ImuMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return ImuMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type { get { return "sensor_msgs/Imu"; } }

        public HeaderMessage header { get; set; }

        public QuaternionMessage orientation { get; set; }
        public double[] orientation_covariance { get; set; }
        public Vector3Message angular_velocity { get; set; }
        public double[] angular_velocity_covariance { get; set; }
        //  Row major about x, y, z axes
        public Vector3Message linear_acceleration { get; set; }
        public double[] linear_acceleration_covariance { get; set; }
        //  Row major x, y z 

        public ImuMessage() {
            this.header = new HeaderMessage();
            this.orientation = new QuaternionMessage();
            this.orientation_covariance = new double[9];
            this.angular_velocity = new Vector3Message();
            this.angular_velocity_covariance = new double[9];
            this.linear_acceleration = new Vector3Message();
            this.linear_acceleration_covariance = new double[9];
        }

        public ImuMessage(HeaderMessage header, QuaternionMessage orientation, double[] orientation_covariance, Vector3Message angular_velocity, double[] angular_velocity_covariance, Vector3Message linear_acceleration, double[] linear_acceleration_covariance) {
            this.header = header;
            this.orientation = orientation;
            this.orientation_covariance = orientation_covariance;
            this.angular_velocity = angular_velocity;
            this.angular_velocity_covariance = angular_velocity_covariance;
            this.linear_acceleration = linear_acceleration;
            this.linear_acceleration_covariance = linear_acceleration_covariance;
        }

        public void Update() {
            this.header.Update();
        }

    }

    /// <summary>
    /// This message is used to specify a region of interest within an image.
    ///
    /// When used to specify the ROI setting of the camera when the image was
    /// taken, the height and width fields should either match the height and
    /// width fields for the associated image; or height = width = 0
    /// indicates that the full resolution image was captured.
    /// <see>http://docs.ros.org/api/sensor_msgs/html/msg/RegionOfInterest.html</see>
    /// </summary>
    public class RegionOfInterestMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return RegionOfInterestMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type { get { return "sensor_msgs/RegionOfInterest"; } }

        /// <summary>
        /// Leftmost pixel of the ROI
        /// (0 if the ROI includes the left edge of the image)
        /// </summary>
        /// <value></value>
        public uint x_offset { get; set; }

        /// <summary>
        /// Topmost pixel of the ROI
        /// (0 if the ROI includes the top edge of the image)
        /// </summary>
        /// <value></value>
        public uint y_offset { get; set; }

        /// <summary>
        /// Height of ROI
        /// </summary>
        /// <value></value>
        public uint height { get; set; }

        /// <summary>
        /// Width of ROI
        /// </summary>
        /// <value></value>
        public uint width { get; set; }

        /// <summary>
        /// True if a distinct rectified ROI should be calculated from the "raw"
        /// ROI in this message. Typically this should be False if the full image
        /// is captured (ROI not used), and True if a subwindow is captured (ROI
        /// used).
        /// </summary>
        /// <value></value>
        public bool do_rectify { get; set; }

        public RegionOfInterestMessage() {
            this.x_offset = 0;
            this.y_offset = 0;
            this.height = 0;
            this.width = 0;
            this.do_rectify = false;
        }

        public RegionOfInterestMessage(uint x_offset, uint y_offset, uint height, uint width, bool do_rectify) {
            this.x_offset = x_offset;
            this.y_offset = y_offset;
            this.height = height;
            this.width = width;
            this.do_rectify = do_rectify;
        }

    }

    /// <summary>
    /// This message defines meta information for a camera. It should be in a
    /// camera namespace on topic "camera_info" and accompanied by up to five
    /// image topics named:
    /// 
    ///   image_raw - raw data from the camera driver, possibly Bayer encoded
    ///   image            - monochrome, distorted
    ///   image_color      - color, distorted
    ///   image_rect       - monochrome, rectified
    ///   image_rect_color - color, rectified
    /// 
    /// The image_pipeline contains packages (image_proc, stereo_image_proc)
    /// for producing the four processed image topics from image_raw and
    /// camera_info. The meaning of the camera parameters are described in
    /// detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
    /// 
    /// The image_geometry package provides a user-friendly interface to
    /// common operations using this meta information. If you want to, e.g.,
    /// project a 3d point into image coordinates, we strongly recommend
    /// using image_geometry.
    ///
    /// /// If the camera is uncalibrated, the matrices D, K, R, P should be left
    /// zeroed out. In particular, clients may assume that K[0] == 0.0
    /// indicates an uncalibrated camera.
    ///<see>http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html</see>
    /// </summary>
    public class CameraInfoMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return CameraInfoMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type { get { return "sensor_msgs/CameraInfo"; } }

        /// <summary>
        /// Time of image acquisition, camera coordinate frame ID
        /// Header timestamp should be acquisition time of image
        /// Header frame_id should be optical frame of camera
        /// origin of frame should be optical center of camera
        /// +x should point to the right in the image
        /// +y should point down in the image
        /// +z should point into the plane of the image
        /// </summary>
        /// <value></value>
        public HeaderMessage header { get; set; }


        /// <summary>
        /// The image dimensions with which the camera was calibrated. Normally
        /// this will be the full camera resolution in pixels.
        /// </summary>
        /// <value></value>
        public uint height { get; set; }
        public uint width { get; set; }

        /// <summary>
        /// The distortion model used. Supported models are listed in
        /// sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
        /// simple model of radial and tangential distortion - is sufficient.
        /// </summary>
        /// <value></value>
        public string distortion_model { get; set; }

        /// <summary>
        /// The distortion parameters, size depending on the distortion model.
        /// For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
        /// </summary>
        /// <value></value>
        public double[] D { get; set; }

        /// <summary>
        /// Intrinsic camera matrix for the raw (distorted) images.
        ///     [fx  0 cx]
        /// K = [ 0 fy cy]
        ///     [ 0  0  1]
        /// Projects 3D points in the camera coordinate frame to 2D pixel
        /// coordinates using the focal lengths (fx, fy) and principal point
        /// (cx, cy).
        /// </summary>
        /// <value></value>
        public double[] K { get; set; }

        /// <summary>
        /// Rectification matrix (stereo cameras only)
        /// A rotation matrix aligning the camera coordinate system to the ideal
        /// stereo image plane so that epipolar lines in both stereo images are
        /// parallel.
        /// </summary>
        /// <value></value>
        public double[] R { get; set; }

        /// <summary>
        /// Projection/camera matrix
        ///     [fx'  0  cx' Tx]
        /// P = [ 0  fy' cy' Ty]
        ///     [ 0   0   1   0]
        /// By convention, this matrix specifies the intrinsic (camera) matrix
        ///  of the processed (rectified) image. That is, the left 3x3 portion
        ///  is the normal camera intrinsic matrix for the rectified image.
        /// It projects 3D points in the camera coordinate frame to 2D pixel
        ///  coordinates using the focal lengths (fx', fy') and principal point
        ///  (cx', cy') - these may differ from the values in K.
        /// For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
        ///  also have R = the identity and P[1:3,1:3] = K.
        /// For a stereo pair, the fourth column [Tx Ty 0]' is related to the
        ///  position of the optical center of the second camera in the first
        ///  camera's frame. We assume Tz = 0 so both cameras are in the same
        ///  stereo image plane. The first camera always has Tx = Ty = 0. For
        ///  the right (second) camera of a horizontal stereo pair, Ty = 0 and
        ///  Tx = -fx' * B, where B is the baseline between the cameras.
        /// Given a 3D point [X Y Z]', the projection (x, y) of the point onto
        ///  the rectified image is given by:
        ///  [u v w]' = P * [X Y Z 1]'
        ///         x = u / w
        ///         y = v / w
        ///  This holds for both images of a stereo pair.
        /// 
        /// 3x4 row-major matrix
        /// </summary>
        /// <value></value>
        public double[] P { get; set; }

        /// <summary>
        /// Binning refers here to any camera setting which combines rectangular
        ///  neighborhoods of pixels into larger "super-pixels." It reduces the
        ///  resolution of the output image to
        ///  (width / binning_x) x (height / binning_y).
        /// The default values binning_x = binning_y = 0 is considered the same
        ///  as binning_x = binning_y = 1 (no subsampling).
        /// </summary>
        /// <value></value>
        public uint binning_x { get; set; }
        public uint binning_y { get; set; }

        public RegionOfInterestMessage roi { get; set; }
        public CameraInfoMessage() {
            this.header = new HeaderMessage();
            this.height = 0;
            this.width = 0;
            this.distortion_model = "plumb_bob";
            this.D = new double[0];
            this.K = new double[9];
            this.R = new double[9];
            this.P = new double[12];
            this.binning_x = 0;
            this.binning_y = 0;
            this.roi = new RegionOfInterestMessage();
        }

        public CameraInfoMessage(HeaderMessage header, uint height, uint width, string distortion_model, double[] D, double[] K, double[] R, double[] P, uint binning_x, uint binning_y, RegionOfInterestMessage roi) {
            this.header = header;
            this.height = height;
            this.width = width;
            this.distortion_model = distortion_model;
            this.D = D;
            this.K = K;
            this.R = R;
            this.P = P;
            this.binning_x = binning_x;
            this.binning_y = binning_y;
            this.roi = roi;
        }

        public void Update() {
            this.header.Update();
        }

        /// <summary>
        /// Builds up camera info given width, height & fov
        /// <see>https://www.programcreek.com/python/?code=carla-simulator%2Fscenario_runner%2Fscenario_runner-master%2Fsrunner%2Fautoagents%2Fros_agent.py</see>
        /// </summary>
        /// <param name="width">width of camera in pixels</param>
        /// <param name="height">height of camera in pixels</param> 
        /// <param name="fovRadians">Field Of View in radians.</param>
        public void BuildCameraInfo(uint width, uint height, double fovRadians) {
            this.width = width;
            this.height = height;
            this.distortion_model = "plumb_bob";

            double cx = this.width / 2.0;
            double cy = this.height / 2.0;
            // focal length pixels = focal length mm * pixel width / sensor size mm
            //double fx = 37.46 * 640.0 / 36.0;// //360;//37.46;//this.width / (2.0 * System.Math.Tan(fovRadians) * System.Math.PI / 360.0);
            double fx = this.width / System.Math.Tan(fovRadians / 2.0);
            double fy = fx;

            this.K = new double[9] {fx, 0, cx,
                                    0, fy, cy,
                                    0, 0, 1};
            this.D = new double[5] { 0, 0, 0, 0, 0 };
            this.R = new double[9] { 1, 0, 0,
                                     0, 1, 0,
                                     0, 0, 1};
            this.P = new double[12] { fx, 0, cx,
                                      0,  0, fy,
                                      cy, 0, 0,
                                      0,  1, 0};


        }


        /// <summary>
        /// Builds up the camera info given width, height, focal length and sensor size
        /// </summary>
        /// <param name="width">Width of camera in pixels.</param>
        /// <param name="height">Height of camera in pixels.</param>
        /// <param name="focalLengthMM">Focal length in millimeters.</param>
        /// <param name="sensorSizeXMM">Sensor width in millimeters.</param>
        /// <param name="sensorSizeYMM">Sensor height in millimeters.</param>
        public void BuildCameraInfo(uint width, uint height, double focalLengthMM, double sensorSizeXMM, double sensorSizeYMM) {
            this.width = width;
            this.height = height;
            this.distortion_model = "plumb_bob";

            double cx = this.width / 2.0;
            double cy = this.height / 2.0;
            // focal length pixels = focal length mm * pixel width / sensor size mm
            //double fx = 37.46 * 640.0 / 36.0;// //360;//37.46;//this.width / (2.0 * System.Math.Tan(fovRadians) * System.Math.PI / 360.0);
            double fx = focalLengthMM * this.width / sensorSizeXMM;
            double fy = focalLengthMM * this.height / sensorSizeYMM;

            this.K = new double[9] {fx, 0, cx,
                                    0, fy, cy,
                                    0, 0, 1};
            this.D = new double[5] { 0, 0, 0, 0, 0 };
            this.R = new double[9] { 1, 0, 0,
                                     0, 1, 0,
                                     0, 0, 1};
            this.P = new double[12] { fx, 0, cx,
                                      0,  0, fy,
                                      cy, 0, 0,
                                      0,  1, 0};


        }

    }
}