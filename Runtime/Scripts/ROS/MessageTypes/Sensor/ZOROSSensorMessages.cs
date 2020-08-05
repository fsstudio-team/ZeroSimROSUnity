
using ZO.ROS.MessageTypes.Std;

namespace ZO.ROS.MessageTypes.Sensor {

    /// <summary>
    ///  This message contains an uncompressed image
    ///  (0, 0) is at top-left corner of image
    /// </summary>
    public class ImageMessage : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "sensor_msgs/Image"; } }

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

    public class LaserScanMessage : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "sensor_msgs/LaserScan"; } }

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
}