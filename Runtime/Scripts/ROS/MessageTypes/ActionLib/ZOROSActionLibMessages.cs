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
}