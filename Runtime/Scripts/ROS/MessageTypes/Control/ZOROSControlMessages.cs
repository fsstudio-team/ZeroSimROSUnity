using ZO.ROS.MessageTypes.Std;
using ZO.ROS.MessageTypes.Trajectory;
using ZO.ROS.MessageTypes.ActionLib;

namespace ZO.ROS.MessageTypes.Control {

    /// <summary>
    /// The tolerances specify the amount the position, velocity, and
    /// accelerations can vary from the setpoints.  For example, in the case
    /// of trajectory control, when the actual position varies beyond
    /// (desired position + position tolerance), the trajectory goal may
    /// abort.
    /// 
    /// There are two special values for tolerances:
    ///  * 0 - The tolerance is unspecified and will remain at whatever the default is
    ///  * -1 - The tolerance is "erased".  If there was a default, the joint will be
    ///         allowed to move without restriction.
    /// </summary>
    public class JointToleranceMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return JointToleranceMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "control_msgs/JointTolerance";

        /// <summary>
        /// in radians or meters (for a revolute or prismatic joint, respectively)
        /// </summary>
        /// <value></value>
        public double position { get; set; }

        /// <summary>
        /// in rad/sec or m/sec
        /// </summary>
        /// <value></value>
        public double velocity { get; set; }

        /// <summary>
        /// in rad/sec^2 or m/sec^2
        /// </summary>
        /// <value></value>
        public double acceleration { get; set; }

        public JointToleranceMessage() {
            this.position = 0;
            this.velocity = 0;
            this.acceleration = 0;
        }

        public JointToleranceMessage(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

    }



    /// <summary>
    /// <see>http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryGoal.html</see>
    /// </summary>
    public class FollowJointTrajectoryGoal : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return FollowJointTrajectoryGoal.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "control_msgs/FollowJointTrajectoryGoal";

        /// <summary>
        /// The joint trajectory to follow
        /// </summary>
        /// <value></value>
        public JointTrajectoryMessage trajectory { get; set; }

        /// <summary>
        /// Tolerances for the trajectory.  If the measured joint values fall
        /// outside the tolerances the trajectory goal is aborted.  Any
        /// tolerances that are not specified (by being omitted or set to 0) are
        /// set to the defaults for the action server (often taken from the
        /// parameter server).
        ///
        /// Tolerances applied to the joints as the trajectory is executed.  If
        /// violated, the goal aborts with error_code set to
        /// PATH_TOLERANCE_VIOLATED.
        /// </summary>
        /// <value></value>
        public JointToleranceMessage[] path_tolerance { get; set; }


        /// <summary>
        ///  To report success, the joints must be within goal_tolerance of the
        /// final trajectory value.  The goal must be achieved by time the
        /// trajectory ends plus goal_time_tolerance.  (goal_time_tolerance
        /// allows some leeway in time, so that the trajectory goal can still
        /// succeed even if the joints reach the goal some time after the
        /// precise end time of the trajectory).
        ///
        /// If the joints are not within goal_tolerance after "trajectory finish
        /// time" + goal_time_tolerance, the goal aborts with error_code set to
        // GOAL_TOLERANCE_VIOLATED
        /// </summary>
        /// <value></value>
        public JointToleranceMessage[] goal_tolerance { get; set; }

        public DurationMessage goal_time_tolerance { get; set; }

        public FollowJointTrajectoryGoal() {
            this.trajectory = new JointTrajectoryMessage();
            this.path_tolerance = new JointToleranceMessage[0];
            this.goal_tolerance = new JointToleranceMessage[0];
            this.goal_time_tolerance = new DurationMessage();
        }

        public FollowJointTrajectoryGoal(JointTrajectoryMessage trajectory, JointToleranceMessage[] path_tolerance, JointToleranceMessage[] goal_tolerance, DurationMessage goal_time_tolerance) {
            this.trajectory = trajectory;
            this.path_tolerance = path_tolerance;
            this.goal_tolerance = goal_tolerance;
            this.goal_time_tolerance = goal_time_tolerance;
        }


        public void Update() {
            trajectory.Update();
        }

    }

    public class FollowJointTrajectoryActionGoal : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return FollowJointTrajectoryActionGoal.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "control_msgs/FollowJointTrajectoryActionGoal";

        public HeaderMessage header { get; set; }


        public FollowJointTrajectoryGoal goal { get; set; }

        public GoalIDMessage goal_id { get; set; }

        public FollowJointTrajectoryActionGoal() {
            this.header = new HeaderMessage();
            this.goal = new FollowJointTrajectoryGoal();
            this.goal_id = new GoalIDMessage();
        }
        public FollowJointTrajectoryGoal(HeaderMessage header, FollowJointTrajectoryGoal goal, GoalIDMessage goal_id) {
            this.header = header;
            this.goal = goal;
            this.goal_id = goal_id;
        }
        public void Update() {
            header.Update();
            goal.Update();
        }

    }

 control_msgs/FollowJointTrajectoryActionResult.msg
 
}