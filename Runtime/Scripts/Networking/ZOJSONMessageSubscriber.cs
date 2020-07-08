using System.Net.Sockets;
using System;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;


namespace ZO.Networking {
    public class ZOJSONMessageSubscriber {
        private TcpClient _tcpClient;
        public Func<ZOJSONMessageSubscriber, string, Task> OnPublishDelegate { get; set; }
        public Func<ZOJSONMessageSubscriber, Task> OnConnectedToPublisherDelegate { get; set; }
        public Func<ZOJSONMessageSubscriber, Task> OnDisconnectedFromPublisherDelegate { get; set; }

        public string Hostname { get; set; } = "localhost";
        public int Port { get; set; } = 4444;

        public string Topic { get; set; } = "None";

        private bool _isRunning = false;
        public async Task RunAsync() {
            
            _isRunning = true;
            while (_isRunning) {
                _tcpClient = new TcpClient();
                await _tcpClient.ConnectAsync(Hostname, Port);

                if (OnConnectedToPublisherDelegate != null) {
                    await OnConnectedToPublisherDelegate(this);
                }
                Debug.Log("INFO: ZOMessageSubscriber::RunAsync connected...");
                NetworkStream netstream = _tcpClient.GetStream();
                // byte[] buffer = new byte[10240];
                string result;
                using(StreamReader reader = new StreamReader(netstream)) {
                    while ((result = await reader.ReadLineAsync()) != null) {
                        if (OnPublishDelegate != null) {
                            await OnPublishDelegate(this, result);
                        }
                    }
                }
                _tcpClient.Close();
                _tcpClient = null;
                Debug.Log("INFO: Disconnected attempting reconnect...");

            }

            // TODO: try to reconnect

        }

        public void Stop() {
            _isRunning = false;
            if (_tcpClient != null) {
                _tcpClient.Close();
            }

        }


        protected virtual Task OnPublisherConnected(ZOTCPServer tcpServer, TcpClient tcpClient) {
            return Task.CompletedTask;
        }


    }
}