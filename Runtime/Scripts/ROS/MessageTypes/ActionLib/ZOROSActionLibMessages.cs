using ZO.ROS.MessageTypes.Std;

namespace ZO.ROS.MessageTypes.ActionLib {
    public class GoalIDMessage : ZOROSMessageInterface {
        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return GoalIDMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "actionlib_msgs/GoalID";

        /// <summary>
        ///    The stamp should store the time at which this goal was requested.
        ///  It is used by an action server when it tries to preempt all
        ///  goals that were requested before a certain time
        /// </summary>
        /// <value></value>
        public TimeMessage stamp { get; set; }

        /// <summary>
        ///   The id provides a way to associate feedback and
        ///  result message with specific goal requests. The id
        ///  specified must be unique.
        /// </summary>
        /// <value></value>
        public string id { get; set; }

        public GoalIDMessage() {
            this.stamp = new TimeMessage();
            this.id = "";
        }

        public GoalIDMessage(TimeMessage stamp, string id) {
            this.stamp = stamp;
            this.id = id;
        }
    }

    public class GoalStatusMessage : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return GoalStatusMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "actionlib_msgs/GoalStatus";


        public GoalIDMessage goal_id { get; set; }
        public byte status { get; set; }

        public const byte PENDING = 0; //  The goal has yet to be processed by the action server
        public const byte ACTIVE = 1; //  The goal is currently being processed by the action server
        public const byte PREEMPTED = 2; //  The goal received a cancel request after it started executing
        //    and has since completed its execution (Terminal State)
        public const byte SUCCEEDED = 3; //  The goal was achieved successfully by the action server (Terminal State)
        public const byte ABORTED = 4; //  The goal was aborted during execution by the action server due
        //     to some failure (Terminal State)
        public const byte REJECTED = 5; //  The goal was rejected by the action server without being processed,
        //     because the goal was unattainable or invalid (Terminal State)
        public const byte PREEMPTING = 6; //  The goal received a cancel request after it started executing
        //     and has not yet completed execution
        public const byte RECALLING = 7; //  The goal received a cancel request before it started executing,
        //     but the action server has not yet confirmed that the goal is canceled
        public const byte RECALLED = 8; //  The goal received a cancel request before it started executing
        //     and was successfully cancelled (Terminal State)
        public const byte LOST = 9; //  An action client can determine that a goal is LOST. This should not be
        //     sent over the wire by an action server
        // Allow for the user to associate a string with GoalStatus for debugging
        public string text { get; set; }

        public GoalStatusMessage() {
            this.goal_id = new GoalIDMessage();
            this.status = 0;
            this.text = "";
        }

        public GoalStatusMessage(GoalIDMessage goal_id, byte status, string text) {
            this.goal_id = goal_id;
            this.status = status;
            this.text = text;
        }
    }


    /// <summary>
    /// Stores the statuses for goals that are currently being tracked by an action server
    /// <see>http://docs.ros.org/kinetic/api/actionlib_msgs/html/msg/GoalStatusArray.html</see>
    /// </summary>
    public class GoalStatusArrayMessage : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return GoalStatusArrayMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "actionlib_msgs/GoalStatusArray";

        public HeaderMessage header {get; set;}
        
        public GoalStatusMessage[] status_list {get; set;}

        public GoalStatusArrayMessage() {
            this.header = new HeaderMessage();
            this.status_list = new GoalStatusMessage[0];
        }

        public GoalStatusArrayMessage(HeaderMessage header, GoalStatusMessage[] status_list) {
            this.header = header;
            this.status_list = status_list;
        }

        public void Update() {
            this.header.Update();
        }

    }
}