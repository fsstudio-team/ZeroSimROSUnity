using System.Net.Mime;
using System.Diagnostics;
using System.IO;
using System.Net.Sockets;
using System;
using System.Net;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;

namespace ZO.Networking {
    /// <summary>
    /// Publisher of binary message over TCP.  
    /// 
    /// Stream format:
    ///     string: Topic + "\n"
    ///     int32: size of data
    ///     byte: data
    /// </summary>
    public class ZOBinaryMessagePublisher {

        private ZOTCPServer _tcpServer;

        public int SubscriberCount { get { return _tcpServer.ConnectedClientCount; } }

        public IPAddress IPAddress { get; set; } = IPAddress.IPv6Any;
        public int Port { get; set; } = 4444;

        public string Topic { get; set; } = "None";

        public bool Debug { get; set; } = false;

        private ZO.Util.ZOFrequencyCounter _debugPublishFrequencyCounter = new Util.ZOFrequencyCounter();
        private int _debugPublishTick = 0;

        /// <summary>
        /// Starts the message publisher as an Async task
        /// </summary>
        /// <returns>Task</returns>
        public async Task RunAsync() {
            _tcpServer = new ZOTCPServer() {
                IPAddress = IPAddress,
                Port = Port,
                OnConnectedDelegate = OnSubscriberConnected,
                OnConnectionClosedDelegate = OnSubscriberDisconnected
            };

            await _tcpServer.RunAsync();
        }

        /// <summary>
        /// Stops publisher.
        /// </summary>
        public void Stop() {
            _tcpServer.Stop();
        }

        ~ZOBinaryMessagePublisher() {
            Stop();
        }


        /// <summary>
        /// Sends header with format:
        ///     string: Topic + "\n"
        ///     int32: size of data
        /// 
        /// </summary>
        /// <param name="msgSize"></param>
        /// <returns></returns>
        protected async virtual Task SendHeader(uint msgSize) {     

            MemoryStream stream = new MemoryStream();            
            BinaryWriter writer = new BinaryWriter(stream);

            // send topic + EoL
            writer.Write(System.Text.Encoding.ASCII.GetBytes(Topic + "\n"));
            
            // send msg size
            writer.Write(msgSize);
            
            stream.Flush();

            await _tcpServer.SendToAllAsync(stream);
        }

        /// <summary>
        /// Async publish binary message using ArraySegment to hold the data.
        /// </summary>
        /// <param name="data">the data</param>
        /// <returns>Task</returns>
        public async virtual Task Publish(ArraySegment<byte> data) {
            if (Debug) {
                _debugPublishFrequencyCounter.Tick();
                _debugPublishTick++;
                if (_debugPublishTick % 10 == 0) {
                    UnityEngine.Debug.Log("INFO: ZOBinaryMessagePublisher:" + Topic + ":PublishFrequency" + _debugPublishFrequencyCounter.GetHZ().ToString("N2"));
                }
            }
            SendHeader((uint)data.Count);
            await _tcpServer.SendToAllAsync(data);
        }

        /// <summary>
        ///  Async publish binary message using MemoryStream to hold the data.
        /// </summary>
        /// <param name="stream">The data</param>
        /// <returns>Task</returns>
        public async virtual Task Publish(MemoryStream stream) {
            if (Debug) {
                _debugPublishFrequencyCounter.Tick();
                _debugPublishTick++;
                if (_debugPublishTick % 10 == 0) {
                    UnityEngine.Debug.Log("INFO: ZOBinaryMessagePublisher:" + Topic + ":PublishFrequency: " + _debugPublishFrequencyCounter.GetHZ().ToString("N2"));
                }
            }

            SendHeader((uint)stream.Length);
            await _tcpServer.SendToAllAsync(stream);
        }

        /// <summary>
        /// Dummy OnSubscriberConnected delegate for ZOTCPServer
        /// </summary>
        /// <param name="tcpServer"></param>
        /// <param name="tcpClient"></param>
        /// <returns></returns>
        protected virtual Task OnSubscriberConnected(ZOTCPServer tcpServer, TcpClient tcpClient) {
            return Task.CompletedTask;
        }

        /// <summary>
        /// /// Dummy OnSubscriberDisconnected delegate for ZOTCPServer
        /// </summary>
        /// <param name="tcpServer"></param>
        /// <param name="tcpClient"></param>
        /// <returns></returns>
        protected virtual Task OnSubscriberDisconnected(ZOTCPServer tcpServer, TcpClient tcpClient) {
            return Task.CompletedTask;
        }

    }
}
