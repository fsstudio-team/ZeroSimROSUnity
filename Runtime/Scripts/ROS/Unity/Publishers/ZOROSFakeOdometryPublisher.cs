using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;
using ZO.ROS.MessageTypes.Sensor;
using ZO.ROS.MessageTypes.Geometry;
using ZO.ROS.Publisher;
using ZO.ROS.Unity;
using ZO.ROS.MessageTypes.Nav;
using ZO.Math;
using ZO.Document;
using ZO.Sensors;


namespace ZO.ROS.Publisher {

    /// <summary>
    /// Publish fake Odometry by just taking the transform of whatever this script is. 
    /// 
    /// Implemented noise mode: https://blog.lxsang.me/post/id/16
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class ZOROSFakeOdometryPublisher : ZOROSUnityGameObjectBase {

        public bool _applyNoise = false;
        public float _a1 = 0.05f;
        public float _a2 = 15.0f;
        public float _a3 = 0.05f;
        public float _a4 = 0.01f;

        public Rigidbody Rigidbody {
            get {
                return gameObject.GetComponent<Rigidbody>();
            }
        }

        private ZOGaussianNoiseModel _noiseModel = new ZOGaussianNoiseModel();
        private OdometryMessage _currentOdometryMessage = null;
        private OdometryMessage _previousOdometryMessage = null;
        private TransformStampedMessage _transformStampedMessage = new TransformStampedMessage();

        public override string Type {
            get { return "publisher.odometry"; }
        }


        protected override void ZOOnValidate() {
            base.ZOOnValidate();
            if (string.IsNullOrEmpty(ROSTopic) == true) {
                ROSTopic = "/odom";
            }

            if (UpdateRateHz == 0) {
                UpdateRateHz = 20;
            }

        }

        protected override void ZOFixedUpdateHzSynchronized() {
            base.ZOFixedUpdateHzSynchronized();

            // publish odometry
            if (ZOROSBridgeConnection.Instance.IsConnected) {

                // See: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

                _currentOdometryMessage = new OdometryMessage();

                // Publish odom on TF as well
                _transformStampedMessage.header.Update();
                _transformStampedMessage.header.frame_id = ZOROSUnityManager.Instance.WorldFrameId; // connect to the world
                _transformStampedMessage.child_frame_id = "odom";
                _transformStampedMessage.UnityLocalTransform = Rigidbody.transform;
                ZOROSUnityManager.Instance.BroadcastTransform(_transformStampedMessage);


                _currentOdometryMessage.Update(); // update times stamps

                // BUGBUG: not super clear on where the pose should be?
                _currentOdometryMessage.pose.pose.GlobalUnityTransform = Rigidbody.transform;

                // get velocity in /odom frame
                Vector3 linear = Rigidbody.velocity;
                _currentOdometryMessage.twist.twist.angular.z = -Rigidbody.angularVelocity.y; // NOTE: negating velocity?

                float yaw = Rigidbody.transform.localRotation.eulerAngles.y * Mathf.Deg2Rad;
                _currentOdometryMessage.twist.twist.linear.x = Mathf.Cos(yaw) * linear.z + Mathf.Sin(yaw) * linear.x;
                _currentOdometryMessage.twist.twist.linear.y = Mathf.Cos(yaw) * linear.x - Mathf.Sin(yaw) * linear.z;
                // set covariance
                // see: https://robotics.stackexchange.com/questions/15265/ros-gazebo-odometry-issue
                // # Odometry covariances for the encoder output of the robot. These values should
                // # be tuned to your robot's sample odometry data, but these values are a good place
                // # to start
                // pose_covariance_diagonal : [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]
                // twist_covariance_diagonal: [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]
                _currentOdometryMessage.pose.covariance[0] = 1e-3;
                _currentOdometryMessage.pose.covariance[7] = 1e-3;
                _currentOdometryMessage.pose.covariance[14] = 1e6;

                _currentOdometryMessage.pose.covariance[21] = 1e6;
                _currentOdometryMessage.pose.covariance[28] = 1e6;
                _currentOdometryMessage.pose.covariance[35] = 1e3;

                _currentOdometryMessage.twist.covariance[0] = 1e-3;
                _currentOdometryMessage.twist.covariance[7] = 1e-3;
                _currentOdometryMessage.twist.covariance[14] = 1e6;
                _currentOdometryMessage.twist.covariance[21] = 1e6;
                _currentOdometryMessage.twist.covariance[28] = 1e6;
                _currentOdometryMessage.twist.covariance[35] = 1e3;

                // BUGBUG: not super clear on this being a child of map?
                _currentOdometryMessage.header.frame_id = ZOROSUnityManager.Instance.WorldFrameId;
                _currentOdometryMessage.child_frame_id = GetComponent<ZOSimOccurrence>().Name;

                if (_applyNoise) {
                    if (_previousOdometryMessage != null) {
                        ApplyNoise(ref _currentOdometryMessage, _previousOdometryMessage);
                    }
                }

                ZOROSBridgeConnection.Instance.Publish<OdometryMessage>(_currentOdometryMessage, ROSTopic);
                _previousOdometryMessage = _currentOdometryMessage;
            }

        }




        private void Initialize() {
            // advertise
            ROSBridgeConnection.Advertise(ROSTopic, OdometryMessage.Type);

        }

        public override void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeConnected");
            Initialize();

        }

        public override void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager) {
            Debug.Log("INFO: ZOImagePublisher::OnROSBridgeDisconnected");
            ROSBridgeConnection?.UnAdvertise(ROSTopic);
        }

        /// <summary>
        /// Apply noise to OdometryMessage.
        /// 
        /// See: https://blog.lxsang.me/post/id/16
        /// </summary>
        /// <param name="currentOdomMessage"></param>
        /// <param name="previousOdomMessage"></param>
        private void ApplyNoise(ref OdometryMessage currentOdomMessage, OdometryMessage previousOdomMessage) {

            double dx = currentOdomMessage.pose.pose.position.x - previousOdomMessage.pose.pose.position.x;
            double dy = currentOdomMessage.pose.pose.position.y - previousOdomMessage.pose.pose.position.y;
            double trans = System.Math.Sqrt(dx * dx + dy * dy);
            QuaternionMessage quaternionMessage = previousOdomMessage.pose.pose.orientation;
            Quaternion q = quaternionMessage.UnityQuaternion;
            double r = q.eulerAngles.z * Mathf.Deg2Rad;
            double p = q.eulerAngles.x * Mathf.Deg2Rad;
            double theta1 = q.eulerAngles.y * Mathf.Deg2Rad;
            q = currentOdomMessage.pose.pose.orientation.UnityQuaternion;
            double theta2 = q.eulerAngles.y * Mathf.Deg2Rad;

            double rot1 = System.Math.Atan2(dy, dx) - theta1;
            double rot2 = theta2 - theta1 - rot1;

            double sd_rot1 = _a1 * System.Math.Abs(rot1) + (_a2 * Mathf.Deg2Rad) * trans;
            double sd_rot2 = _a1 * System.Math.Abs(rot2) + (_a2 * Mathf.Deg2Rad) * trans;
            double sd_trans = _a3 * trans + _a4 * (System.Math.Abs(rot1) + System.Math.Abs(rot2));

            _noiseModel.StdDev = sd_trans * sd_trans;
            trans = _noiseModel.Apply(trans);

            _noiseModel.StdDev = sd_rot1 * sd_rot1;
            rot1 = _noiseModel.Apply(rot1);

            _noiseModel.StdDev = sd_rot2 * sd_rot2;
            rot2 = _noiseModel.Apply(rot2);

            currentOdomMessage.pose.pose.position.x += trans * System.Math.Cos(theta1 + rot1);
            currentOdomMessage.pose.pose.position.y += trans * System.Math.Sin(theta1 + rot1);
            
            theta2 += (rot1 + rot2);

            Quaternion yawQuaternion = Quaternion.EulerAngles(0, (float)theta2, 0);
            currentOdomMessage.pose.pose.orientation.FromUnityQuaternion(yawQuaternion);

        }


    }
}