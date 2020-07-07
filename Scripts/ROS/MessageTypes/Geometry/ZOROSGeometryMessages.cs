using System.Numerics;

using ZO.ROS.MessageTypes.Std;

namespace ZO.ROS.MessageTypes.Geometry {

    public class Vector3Message : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "geometry_msgs/Vector3"; } }
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public Vector3Message() {
            x = 0;
            y = 0;
            z = 0;
        }

        /// <summary>
        /// Convert from Unity Vector3 to ROS Vector3 and convert coordinate system:
        /// 
        /// ROS Coordinate System (See: https://www.ros.org/reps/rep-0103.html)
        /// x forward
        /// y left 
        /// z up
        /// 
        /// Unity Coordinate System:
        /// x right
        /// y up
        /// z forward
        /// </summary>
        /// <param name="v">Unity Vector3</param>
        public void FromUnityVector3(UnityEngine.Vector3 v) {
            this.x = (double)v.z;
            this.y = -(double)v.x;
            this.z = (double)v.y;
        }



        /// <summary>
        /// Convert from ROS Vector3 to Unity Vector3 and convert coordinate system:
        /// 
        /// ROS Coordinate System (See: https://www.ros.org/reps/rep-0103.html)
        /// x forward
        /// y left 
        /// z up
        /// 
        /// Unity Coordinate System:
        /// x right
        /// y up
        /// z forward        
        /// </summary>
        /// <returns>Unity Vector3</returns>
        public UnityEngine.Vector3 ToUnityVector3() {
            return new UnityEngine.Vector3((float)-this.x, (float)this.z, (float)this.y);
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Vector3 UnityVector3 {
            get { return ToUnityVector3(); }
            set { FromUnityVector3(value); }
        }
    }


    /// <summary>
    /// This represents an orientation in free space in quaternion form.
    /// See: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html
    /// </summary>
    public class QuaternionMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "geometry_msgs/Quaternion"; } }

        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public double w { get; set; }
        public QuaternionMessage() {
            x = 0;
            y = 0;
            z = 0;
            w = 0;
        }

        /// <summary>
        /// Convert from ROS Quaternion to Unity Quaternion.  Also converts coordinate system.
        /// 
        /// ROS Coordinate System (See: https://www.ros.org/reps/rep-0103.html)
        /// x forward
        /// y left 
        /// z up
        /// 
        /// Unity Coordinate System:
        /// x right
        /// y up
        /// z forward                
        /// </summary>
        /// <returns>Unity Quaternion</returns>
        public UnityEngine.Quaternion ToUnityQuaternion() {
            return new UnityEngine.Quaternion((float)-this.x, (float)-this.y, (float)-this.z, (float)this.w);
        }

        /// <summary>
        /// Convert from Unity Quaternion to ROS Quaternoin.abstract  Also converts coordinate system.abstract
        /// 
        /// ROS Coordinate System (See: https://www.ros.org/reps/rep-0103.html)
        /// x forward
        /// y left 
        /// z up
        /// 
        /// Unity Coordinate System:
        /// x right
        /// y up
        /// z forward                
        /// </summary>
        /// <param name="q"></param>
        public void FromUnityQuaternion(UnityEngine.Quaternion q) {
            this.x = (double)-q.x;
            this.y = (double)-q.z;
            this.z = (double)-q.y;
            this.w = (double)q.w;
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Quaternion UnityQuaternion {
            get { return ToUnityQuaternion(); }
            set { FromUnityQuaternion(value); }
        }
    }

    /// <summary>
    /// This expresses velocity in free space broken into its linear and angular parts.
    /// See: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html
    /// </summary>
    public class TwistMessage : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "geometry_msgs/Twist"; } }

        public Vector3Message linear { get; set; }
        public Vector3Message angular { get; set; }

        public TwistMessage() {
            this.linear = new Vector3Message();
            this.angular = new Vector3Message();
        }

        public TwistMessage(Vector3Message linear, Vector3Message angular) {
            this.linear = linear;
            this.angular = angular;
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Vector3 UnityLinear {
            get { return this.linear.UnityVector3; }
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Vector3 UnityAngular {
            get { return this.angular.UnityVector3; }
        }

    }


    /// <summary>
    /// This expresses velocity in free space with uncertainty.
    /// See: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistWithCovariance.html
    /// </summary>
    public class TwistWithCovarianceMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "geometry_msgs/TwistWithCovariance"; } }

        //  This expresses velocity in free space with uncertainty.
        public TwistMessage twist { get; set; }
        //  Row-major representation of the 6x6 covariance matrix
        //  The orientation parameters use a fixed-axis representation.
        //  In order, the parameters are:
        //  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        public double[] covariance { get; set; }

        public TwistWithCovarianceMessage() {
            this.twist = new TwistMessage();
            this.covariance = new double[36];
        }

        public TwistWithCovarianceMessage(TwistMessage twist, double[] covariance) {
            this.twist = twist;
            this.covariance = covariance;
        }

    }

    /// <summary>
    /// This represents the transform between two coordinate frames in free space.
    /// See: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Transform.html
    /// </summary>
    public class TransformMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "geometry_msgs/Transform"; } }

        public Vector3Message translation { get; set; }
        public QuaternionMessage rotation { get; set; }
        public TransformMessage() {
            translation = new Vector3Message();
            rotation = new QuaternionMessage();
        }

        /// <summary>
        /// Convert local Unity Transform to ROS Transform. 
        /// Does Unity to ROS coordinate system transform.
        /// </summary>
        /// <param name="transform"></param>
        public void FromLocalUnityTransform(UnityEngine.Transform transform) {
            this.translation.UnityVector3 = transform.localPosition;
            this.rotation.UnityQuaternion = transform.localRotation;
        }

        /// <summary>
        /// Convert global Unity Transform to ROS Transform. 
        /// Does Unity to ROS coordinate system transform.
        /// </summary>
        /// <param name="transform"></param>
        public void FromGlobalUnityTransform(UnityEngine.Transform transform) {
            translation.UnityVector3 = transform.position;
            rotation.UnityQuaternion = transform.rotation;
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Transform LocalUnityTransform {
            set { FromLocalUnityTransform(value); }
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Transform GlobalUnityTransform {
            set { FromGlobalUnityTransform(value); }
        }

    }

    /// <summary>
    /// # This expresses a transform from coordinate frame header.frame_id
    /// to the coordinate frame child_frame_id
    ///
    /// This message is mostly used by the 
    /// tf package. 
    /// See its documentation for more information.
    /// See: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TransformStamped.html
    /// </summary>
    public class TransformStampedMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "geometry_msgs/TransformStamped"; } }

        public string child_frame_id { get; set; }
        public HeaderMessage header { get; set; }
        public TransformMessage transform { get; set; }
        public TransformStampedMessage() {
            header = new HeaderMessage();
            child_frame_id = "";
            transform = new TransformMessage();
        }

        /// <summary>
        /// Convert local Unity Transform to ROS Transform. 
        /// Does Unity to ROS coordinate system transform.
        /// </summary>
        /// <param name="transform"></param>
        public void FromLocalUnityTransform(UnityEngine.Transform transform) {
            this.transform.LocalUnityTransform = transform;
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Transform UnityLocalTransform {
            set { FromLocalUnityTransform(value); }
        }

        /// <summary>
        /// Convert global Unity Transform to ROS Transform. 
        /// Does Unity to ROS coordinate system transform.
        /// </summary>
        /// <param name="transform"></param>
        public void FromGlobalUnityTransform(UnityEngine.Transform transform) {
            this.transform.GlobalUnityTransform = transform;
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Transform GlobalUnityTransform {
            set { FromGlobalUnityTransform(value); }
        }

    }

    /// <summary>
    /// This contains the position of a point in free space
    /// See: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Point.html
    /// </summary>
    public class PointMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "geometry_msgs/Point"; } }

        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }

        public PointMessage() {
            this.x = 0.0;
            this.y = 0.0;
            this.z = 0.0;
        }

        public PointMessage(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        /// <summary>
        /// Convert from Unity Vector3 to ROS Vector3 and convert coordinate system:
        /// 
        /// ROS Coordinate System (See: https://www.ros.org/reps/rep-0103.html)
        /// x forward
        /// y left 
        /// z up
        /// 
        /// Unity Coordinate System:
        /// x right
        /// y up
        /// z forward
        /// </summary>
        /// <param name="v">Unity Vector3</param>
        public void FromUnityVector3(UnityEngine.Vector3 v) {
            this.x = (double)v.z;
            this.y = (double)-v.x;
            this.z = (double)v.y;
        }



        /// <summary>
        /// Convert from ROS Vector3 to Unity Vector3 and convert coordinate system:
        /// 
        /// ROS Coordinate System (See: https://www.ros.org/reps/rep-0103.html)
        /// x forward
        /// y left 
        /// z up
        /// 
        /// Unity Coordinate System:
        /// x right
        /// y up
        /// z forward        
        /// </summary>
        /// <returns>Unity Vector3</returns>
        public UnityEngine.Vector3 ToUnityVector3() {
            return new UnityEngine.Vector3((float)-this.x, (float)this.z, (float)this.y);
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Vector3 UnityVector3 {
            get { return ToUnityVector3(); }
            set { FromUnityVector3(value); }
        }
    }


    /// <summary>
    /// A representation of pose in free space, composed of position and orientation. 
    /// See: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html
    /// </summary>
    public class PoseMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "geometry_msgs/Pose"; } }

        //  A representation of pose in free space, composed of position and orientation. 
        public PointMessage position { get; set; }
        public QuaternionMessage orientation { get; set; }

        public PoseMessage() {
            this.position = new PointMessage();
            this.orientation = new QuaternionMessage();
        }

        public PoseMessage(PointMessage position, QuaternionMessage orientation) {
            this.position = position;
            this.orientation = orientation;
        }

        /// <summary>
        /// Convert Unity Transform to ROS Pose. 
        /// Does Unity to ROS coordinate system transform.
        /// </summary>
        /// <param name="transform"></param>
        public void FromLocalUnityTransform(UnityEngine.Transform transform) {
            this.position.UnityVector3 = transform.localPosition;
            this.orientation.UnityQuaternion = transform.localRotation;
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Transform LocalUnityTransform {
            set { FromLocalUnityTransform(value); }
        }

        /// <summary>
        /// Convert global Unity Transform to ROS Pose. 
        /// Does Unity to ROS coordinate system transform.
        /// </summary>
        /// <param name="transform"></param>
        public void FromGlobalUnityTransform(UnityEngine.Transform transform) {
            this.position.UnityVector3 = transform.position;
            this.orientation.UnityQuaternion = transform.rotation;
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Transform GlobalUnityTransform {
            set { FromGlobalUnityTransform(value); }
        }

    }

    /// <summary>
    /// This represents a pose in free space with uncertainty.
    /// See: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html
    /// </summary>
    public class PoseWithCovarianceMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return "geometry_msgs/PoseWithCovariance"; } }

        public const string RosMessageName = "geometry_msgs/PoseWithCovariance";

        //  This represents a pose in free space with uncertainty.
        public PoseMessage pose { get; set; }
        //  Row-major representation of the 6x6 covariance matrix
        //  The orientation parameters use a fixed-axis representation.
        //  In order, the parameters are:
        //  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        public double[] covariance { get; set; }

        public PoseWithCovarianceMessage() {
            this.pose = new PoseMessage();
            this.covariance = new double[36];
        }

        public PoseWithCovarianceMessage(PoseMessage pose, double[] covariance) {
            this.pose = pose;
            this.covariance = covariance;
        }

        /// <summary>
        /// Convert local Unity Transform to ROS Pose. 
        /// Does Unity to ROS coordinate system transform.
        /// </summary>
        /// <param name="transform"></param>
        public void FromLocalUnityTransform(UnityEngine.Transform transform) {
            this.pose.LocalUnityTransform = transform;
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Transform LocalUnityTransform {
            set { FromLocalUnityTransform(value); }
        }

        /// <summary>
        /// Convert global Unity Transform to ROS Pose. 
        /// Does Unity to ROS coordinate system transform.
        /// </summary>
        /// <param name="transform"></param>
        public void FromGlobalUnityTransform(UnityEngine.Transform transform) {
            this.pose.GlobalUnityTransform = transform;
        }

        [Newtonsoft.Json.JsonIgnore]
        public UnityEngine.Transform GlobalUnityTransform {
            set { FromGlobalUnityTransform(value); }
        }

    }
}