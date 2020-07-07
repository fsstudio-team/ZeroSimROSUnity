using System.Net.Sockets;
using System;
using System.IO;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;


namespace ZO.Networking {
    /// <summary>
    /// Subscribes to arbitrary binary messages.
    /// Stream format:
    ///     string: Topic + "\n"
    ///     int32: size of data
    ///     byte: data
    /// </summary>
    public class ZOBinaryMessageSubscriber {
        private TcpClient _tcpClient;

        /// <summary>
        /// Delegate for when message is received.  
        /// Parameters: this, byte array of binary message
        /// Returns: Async task
        /// </summary>
        /// <value></value>
        public Func<ZOBinaryMessageSubscriber, byte[], Task> OnMessageReceivedDelegate { get; set; }

        /// <summary>
        /// Delegate for when we are connected to a publisher.
        /// Parameters: this
        /// Returns: Async Task
        /// </summary>
        /// <value></value>
        public Func<ZOBinaryMessageSubscriber, Task> OnConnectedToPublisherDelegate { get; set; }


        /// <summary>
        /// Delegate for when we are disconnected from a publisher.
        /// Parameters: this
        /// Returns: Async Task
        /// </summary>
        /// <value></value>
        public Func<ZOBinaryMessageSubscriber, Task> OnDisconnectedFromPublisherDelegate { get; set; }

        public string Hostname { get; set; } = "localhost";
        public int Port { get; set; } = 4444;

        public string Topic { get; set; } = "None";

        private bool _isRunning = false;
        private int _messageLength = -1;
        private byte[] _messageBuffer;

        /// <summary>
        /// Run asynchronously
        /// </summary>
        /// <returns></returns>
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
                bool gotTopic = false;
                int topicIndex = 0;
                byte[] topicBytes = Encoding.ASCII.GetBytes(Topic + "\n");
                while (_isRunning) {

                    if (gotTopic == false) {
                        byte[] single_buf = new byte[1];
                        int bytesRead = await netstream.ReadAsync(single_buf, 0, 1);
                        if (single_buf[0] == topicBytes[topicIndex]) {
                            topicIndex++;
                            if (topicIndex == topicBytes.Length) {
                                gotTopic = true;
                                topicIndex = 0;
                            }
                        } else {
                            topicIndex = 0; // restart
                        }
                    } else {
                        byte[] buf = new byte[4];
                        int bytesRead = await netstream.ReadAsync(buf, 0, 4);
                        _messageLength = BitConverter.ToInt32(buf, 0);
                        // Debug.Log(_messageLength);
                        _messageBuffer = new byte[_messageLength];
                        bytesRead = await netstream.ReadAsync(_messageBuffer, 0, _messageLength);

                        if (bytesRead == _messageLength) {
                            if (OnMessageReceivedDelegate != null) {
                                await OnMessageReceivedDelegate(this, _messageBuffer);
                            }
                            // search for topic again
                            gotTopic = false;
                            topicIndex = 0;

                        } else {
                            // ERROR:
                            _isRunning = false;
                            Debug.Log("ERROR: bytesRead != _messageLength");
                        }

                    }
                }
                netstream.Close();
                _tcpClient.Close();
                _tcpClient = null;
                Debug.Log("INFO: Disconnected...");
                if (OnDisconnectedFromPublisherDelegate != null) {
                    await OnDisconnectedFromPublisherDelegate(this);
                }

            }

        }

        /// <summary>
        /// Stops subscription
        /// </summary>
        public void Stop() {
            _isRunning = false;
            if (_tcpClient != null) {
                _tcpClient.Close();
            }
        }

    }
}