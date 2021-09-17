using System.Collections.ObjectModel;
using UnityEngine;
using System.Collections.Generic;


namespace ZO.Util {
    [CreateAssetMenu(fileName = "DockerRun", menuName = "ZeroSim/ZODockerRunParameters", order = 1)]
    public class ZODockerRunParameters : ScriptableObject {

        /// <summary>
        /// The Docker image name.
        /// </summary>
        public string imageName = "zerosim_ros";

        /// <summary>
        /// The container name of the docker
        /// </summary>
        public string containerName = "my_docker_container";

        /// <summary>
        /// The command to send to the Docker image.
        /// </summary>
        public string command = "roslaunch zero_sim_ros basic_unity_editor.launch";

        /// <summary>
        /// Volumes to mount.
        /// </summary>
        // public List<ZODocker.VolumeMapEntry> volumes = null;

        /// <summary>
        /// Ports to expose.
        /// </summary>
        public List<int> ports = null;

        /// <summary>
        /// Environment variables to pass to the Docker image.
        /// </summary>
        public List<string> environments = null;

        /// <summary>
        /// Run X11 server.  Note:  Will require NVidia GPU.
        /// </summary>
        public bool setupX11 = false;

        /// <summary>
        /// Do any ROS setup required before running the command.
        /// </summary>
        public bool setupROS = false;

        /// <summary>
        /// Do any VNC setup required before running the command.
        /// </summary>
        public bool setupVNC = false;

        /// <summary>
        /// Show docker output.
        /// </summary>
        public bool showOutput = false;

        /// <summary>
        /// Run docker remotely using the Docker Engine API.
        /// <see>https://docs.docker.com/engine/api/v1.40/#</see>
        /// </summary>
        public bool runRemoteDocker = false;

    
        /// <summary>
        /// The remote docker URI.
        /// Note to connect to the local docker:
        /// * Linux & MacOS use: `unix:///var/run/docker.sock`
        /// * Windows use: `npipe://./pipe/docker_engine`
        /// * MacOs use: 
        /// </summary>
        public string remoteDockerUri = "unix:///var/run/docker.sock";

    }

}
