using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;  // ONLY for debug log

namespace ZO.Networking {
    /// <summary>
    /// A TCP server.
    /// </summary>
    public class ZOTCPServer {

        private TcpListener _tcpListener;
        private bool _isListening = false;

        public IPAddress IPAddress { get; set; } = IPAddress.IPv6Any;
        public int Port { get; set; } = 4444;

        /// <summary>
        /// Number of connected clients.
        /// </summary>
        /// <value></value>
        public int ConnectedClientCount {
            get { return _connectedClients.Count; }
        }

        /// <summary>
        /// Delegate for when a client is connected.
        /// Parameters: this, a TcpClient instance
        /// Return: Async Task
        /// </summary>
        /// <value></value>
        public Func<ZOTCPServer, TcpClient, Task> OnConnectedDelegate { get; set; }
        
        /// <summary>
        /// Delegate for when a connection is closed.
        /// Parameters: this, the TcpClient that is disconnected.
        /// Returns: Async Task
        /// </summary>
        /// <value></value>
        public Func<ZOTCPServer, TcpClient, Task> OnConnectionClosedDelegate { get; set; }

        /// <summary>
        /// Delegate for when data is received.
        /// Parameters: this, MemoryStream containing data
        /// Returns: Async Task
        /// </summary>
        /// <value></value>
        public Func<ZOTCPServer, MemoryStream, Task> OnDataReceivedDelegate { get; set; }

        struct ConnectedClient {
            public TcpClient tcpClient;
            public MemoryStream rcvBuffer;
            public Task clientConnectedTask;
            public Task readTask;
        };

        private List<ConnectedClient> _connectedClients = new List<ConnectedClient>();

        ~ZOTCPServer() {
            Stop();
        }

        /// <summary>
        /// Start listening asynchronously
        /// </summary>
        /// <returns></returns>
        public async Task RunAsync() {
            Debug.Log("INFO: ZOTCPServer::RunAsync Start");
            // List<TcpClient> tcpClients = new List<TcpClient>();
            // List<Task> clientConnectTasks = new List<Task>();
            // List<Task> readTasks = new List<Task>();
            try {
                _tcpListener = new TcpListener(IPAddress, Port);

                _isListening = true;
                _tcpListener.Start();

                Debug.Log("INFO: ZOTCPServer::RunAsync listening..." + IPAddress.ToString() + " : " + Port.ToString());


                while (_isListening) {
                    TcpClient tcpClient = await _tcpListener.AcceptTcpClientAsync();
                    ConnectedClient connectedClient = new ConnectedClient();
                    connectedClient.tcpClient = tcpClient;
                    Debug.Log("INFO: ZOTCPServer::RunAsync TCP client connected: "
                            + ((IPEndPoint)tcpClient.Client.RemoteEndPoint).Address.ToString()
                            + " Port: " + ((IPEndPoint)tcpClient.Client.RemoteEndPoint).Port);

                    if (OnConnectedDelegate != null) {
                        Task clientConnectTask = OnConnectedDelegate(this, tcpClient);
                        connectedClient.clientConnectedTask = clientConnectTask;
                    }

                    Task asyncReadTask = Task.Run(() => ClientReadAsync(connectedClient));
                    connectedClient.readTask = asyncReadTask;
                    _connectedClients.Add(connectedClient);
                }
            } catch (System.Exception e) {
                Debug.Log("ERROR: ZOTCPServer::RunAsync Exception: " + e.ToString());
            } finally {
                Debug.Log("INFO: ZOTCPServer::RunAsync Finally");
                // await Task.WhenAll(clientConnectTasks);
                // await Task.WhenAll(readTasks);
                // foreach (TcpClient tcpClient in tcpClients) {
                //     // tcpClient.Close();
                //     tcpClient.Dispose();
                // }
            }

            Debug.Log("INFO: ZOTCPServer::RunAsync End");

        }

        /// <summary>
        /// Stops TcpServer and disconnects all clients.
        /// </summary>
        public void Stop() {
            Debug.Log("INFO: ZOTCPServer::Stop Requested");
            _isListening = false; // stop listening

            // shutdown any connected clients
            foreach (ConnectedClient client in _connectedClients) {
                client.tcpClient.GetStream().Close();
            }
        }

        private async Task ClientReadAsync(ConnectedClient connectedClient) {
            Debug.Log("INFO: ZOTCPServer::ClientReadAsync Start");
            bool isConnected = true;
            NetworkStream netstream = connectedClient.tcpClient.GetStream();
            byte[] buffer = new byte[10240];
            while (isConnected) {
                int bytesRead = -1;
                try {
                    bytesRead = await netstream.ReadAsync(buffer, 0, buffer.Length);
                } catch (System.Exception e) {
                    Debug.Log("ERROR: ZOTCPServer::ClientReadAsync " + e.ToString());
                }

                if (bytesRead > 0) {
                    Debug.Log("INFO: ZOTCPServer::ClientReadAsync Bytes Read: " + bytesRead);
                    if (OnDataReceivedDelegate != null) {
                        connectedClient.rcvBuffer = new MemoryStream(buffer);

                        await OnDataReceivedDelegate(this, connectedClient.rcvBuffer);
                    }
                } else { // connection closed or some error
                    Debug.Log("INFO: ZOTCPServer::ClientReadAsync Connection Closed");
                    isConnected = false;
                    if (OnConnectedDelegate != null) {
                        await OnConnectionClosedDelegate(this, connectedClient.tcpClient);
                    }

                    _connectedClients.Remove(connectedClient);
                }
            }

        }

        /// <summary>
        /// Send data asynchronously to specific client
        /// </summary>
        /// <param name="data">byte array of data</param>
        /// <param name="tcpClient">the tcp client to send data to.</param>
        /// <returns></returns>
        public async Task SendAsync(ArraySegment<byte> data, TcpClient tcpClient) {
            if (tcpClient.Client.Connected) {
                await tcpClient.GetStream().WriteAsync(data.Array, data.Offset, data.Count);
            }
            // return Task.CompletedTask;
        }

        /// <summary>
        /// Send data to all connected clients asynchronously.
        /// </summary>
        /// <param name="data">The data array to send.</param>
        /// <returns></returns>
        public async Task SendToAllAsync(ArraySegment<byte> data) {
            foreach (ConnectedClient client in _connectedClients) {
                await SendAsync(data, client.tcpClient);
            }
        }


        /// <summary>
        /// Send data asynchronously to specific client using MemoryStream as a data container.
        /// </summary>
        /// <param name="stream">the data</param>
        /// <param name="tcpClient">the connected client</param>
        /// <returns></returns>
        public async Task SendAsync(MemoryStream stream, TcpClient tcpClient) {
            if (tcpClient.Client.Connected) {
                byte[] data = stream.ToArray();
                // Debug.Log("INFO: data length: " + data.Length);
                await tcpClient.GetStream().WriteAsync(data, 0, data.Length);
            }
            // return Task.CompletedTask;
        }

        /// <summary>
        /// Send data asynchronously to all connnected clients using MemoryStream as a data container.
        /// </summary>
        /// <param name="stream">the data</param>
        /// <returns></returns>
        public async Task SendToAllAsync(MemoryStream stream) {
            foreach (ConnectedClient client in _connectedClients) {
                await SendAsync(stream, client.tcpClient);
            }
        }

    }
}
