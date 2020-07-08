using System;

namespace ZO.Networking
{
    public class ZOPubSubHeader {
        public string Topic {get; set;}
        public long TimeStampMilliseconds {get; set; }

        public ZOPubSubHeader(string topic_) {
            Topic = topic_;
            TimeStampMilliseconds = DateTimeOffset.Now.ToUnixTimeMilliseconds();
        }

    }
}