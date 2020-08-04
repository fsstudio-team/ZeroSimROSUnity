using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.Geometry;

namespace ZO.ROS.MessageTypes.Trajectory {


    /// <summary>
    /// Each trajectory point specifies either positions[, velocities[, accelerations]]
    /// or positions[, effort] for the trajectory to be executed.
    /// All specified values are in the same order as the joint names in JointTrajectory.msg
    /// </summary>
    public class JointTrajectoryPointMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return JointTrajectoryPointMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "trajectory_msgs/JointTrajectoryPoint";


        public double[] positions { get; set; }
        public double[] velocities { get; set; }
        public double[] accelerations { get; set; }
        public double[] effort { get; set; }
        public DurationMessage time_from_start { get; set; }

        public JointTrajectoryPointMessage() {
            this.positions = new double[0];
            this.velocities = new double[0];
            this.accelerations = new double[0];
            this.effort = new double[0];
            this.time_from_start = new DurationMessage();
        }

        public JointTrajectoryPointMessage(double[] positions, double[] velocities, double[] accelerations, double[] effort, DurationMessage time_from_start) {
            this.positions = positions;
            this.velocities = velocities;
            this.accelerations = accelerations;
            this.effort = effort;
            this.time_from_start = time_from_start;
        }
    }

    /// <summary>
    /// A joint control message.
    /// Used for controllers such as `/arm_control/control` topic.
    /// </summary>
    public class JointTrajectoryMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return JointTrajectoryMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "trajectory_msgs/JointTrajectory";

        public HeaderMessage header { get; set; }
        public string[] joint_names { get; set; }

        public JointTrajectoryPointMessage[] points { get; set; }

        public JointTrajectoryMessage() {
            this.header = new HeaderMessage();
            this.joint_names = new string[0];
            this.points = new JointTrajectoryPointMessage[0];
        }

        public JointTrajectoryMessage(HeaderMessage header, string[] joint_names, JointTrajectoryPointMessage[] points) {
            this.header = header;
            this.joint_names = joint_names;
            this.points = points;
        }

        public void Update() {
            this.header.Update();
        }

    }
}