using ZO.ROS.MessageTypes.Std;

namespace ZO.ROS.MessageTypes.ROSGraph {

    /// <summary>
    /// roslib/Clock is used for publishing simulated time in ROS. 
    /// This message simply communicates the current time.
    /// For more information, see http://www.ros.org/wiki/Clock
    /// </summary>
    public class ClockMessage : ZOROSMessageInterface {

        [Newtonsoft.Json.JsonIgnore]
        public string MessageType { get { return ClockMessage.Type; } }

        [Newtonsoft.Json.JsonIgnore]
        public static string Type = "rosgraph_msgs/Clock.msg";

        public TimeMessage clock { get; set; }

        public ClockMessage() {
            this.clock = new TimeMessage();
            this.clock.Now();
        }

        public ClockMessage(TimeMessage clock) {
            this.clock = clock;
        }

        public void Update() {
            this.clock.Update();
        }

    }


}