using System.Net.Sockets;
using System;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace ZO.Networking {
    public class ZOJSONMessagePublisher {
        private ZOTCPServer _tcpServer;

        public bool IsSubscribersConnected { get; protected set; }

        public IPAddress IPAddress { get; set; } = IPAddress.IPv6Any;
        public int Port { get; set; } = 4444;

        public string Topic { get; set; } = "None";


        public async Task RunAsync() {
            _tcpServer = new ZOTCPServer() {
                IPAddress = IPAddress,
                Port = Port,
                OnConnectedDelegate = OnSubscriberConnected,
                OnConnectionClosedDelegate = OnSubscriberDisconnected
            };

            await _tcpServer.RunAsync();
        }

        public void Stop() {
            _tcpServer.Stop();
        }


        public virtual Task Publish(string jsonString) {
            JObject messagebody = JObject.Parse(jsonString);
            return Publish(messagebody);
        }


        public virtual Task Publish(JObject jsonObj) {
            JObject msg = new JObject();
            msg.Add("header", JObject.FromObject(new ZOPubSubHeader(Topic)));
            msg.Add("body", jsonObj);

            string outJsonString = msg.ToString(Formatting.None) + "\n";
            ArraySegment<byte> publishArray = new ArraySegment<byte>(Encoding.ASCII.GetBytes(outJsonString));
            Task publish = _tcpServer.SendToAllAsync(publishArray);
            return publish;

        }


        protected virtual Task OnSubscriberConnected(ZOTCPServer tcpServer, TcpClient tcpClient) {
            IsSubscribersConnected = true;
            return Task.CompletedTask;
        }

        protected virtual Task OnSubscriberDisconnected(ZOTCPServer tcpServer, TcpClient tcpClient) {
            // BUGBUG: if multiple subscribers connected IsSubscribersConnected can be still true
            // TODO: Fixme 
            IsSubscribersConnected = false;
            return Task.CompletedTask;
        }

    }
}