using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using System;
using System.Linq;

using Docker.DotNet;
using Docker.DotNet.Models;

namespace ZO.Util {
    public static class ZODocker {

        [System.Serializable]
        public class VolumeMapEntry {
            public string hostDirectory;
            public string dockerDirectory;

            public string String() {
                return $" --volume={this.hostDirectory}:{this.dockerDirectory}";
            }
        }

        /// <summary>
        /// Docker run with special ROS specific parameters.
        /// <see>https://docs.docker.com/engine/reference/commandline/run/</see>
        /// </summary>
        /// <param name="imageName"></param>
        /// <param name="containerName"></param>
        /// <param name="command"></param>
        /// <param name="volumes"></param>
        /// <param name="ports"></param>
        /// <param name="environments"></param>
        /// <param name="runX11"></param>
        /// <param name="setupROS"></param>
        /// <param name="showOutput"></param>
        public static void DockerRunROS(string imageName,
                             string containerName,
                             string command,
                             List<VolumeMapEntry> volumes = null,
                             List<int> ports = null,
                             List<string> environments = null,
                             bool runX11 = false,
                             bool setupROS = false,
                             bool showOutput = false) {

            // find the docker executable
            int exitCode = -99;
            string dockerExecutable = ZOSystem.RunProcessAndGetOutput("./", "which", "docker", out exitCode);

            // TODO: check exit code...

            // remove \n and spaces
            dockerExecutable = Regex.Replace(dockerExecutable, @"\s+", string.Empty);


            // build volumes string
            string dockerVolString = "";
            if (volumes != null) {
                foreach (VolumeMapEntry volumeMapEntry in volumes) {
                    dockerVolString += volumeMapEntry.String();
                }
            }

            string portsString = "";
            if (ports != null) {
                foreach (int port in ports) {
                    portsString += $" --publish={port}:{port}";
                }
            }

            string dockerEnvString = "";
            if (environments != null) {
                foreach (string env in environments) {
                    dockerEnvString += $" --env={env}";
                }
            }

            // docker run string
            // See: https://docs.docker.com/engine/reference/commandline/run/
            string dockerRunString = $"docker run --name={containerName} --rm ";


            // source ROS setup
            string preDockerRunString = "";

            // build up necessary stuff for X11
            if (runX11 == true) {
                string tmpDirectory = Path.GetTempPath();
                // setup XAuthority in order for X11 + OpenGL to work
                preDockerRunString = $@"
{preDockerRunString}                                
XAUTH={tmpDirectory}.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
";
                dockerEnvString += " --env=DISPLAY=$DISPLAY";
                dockerEnvString += " --env=QT_X11_NO_MITSHM=1";
                dockerEnvString += " --env=XAUTHORITY=$XAUTH";
                dockerVolString += " --volume=$XAUTH:$XAUTH";
                dockerVolString += $" --volume={tmpDirectory}.X11-unix:{tmpDirectory}.X11-unix:rw";
                dockerRunString += " --gpus all";
            }


            string runCommandString = "";

            if (setupROS == true) {
                // sets up the ROS bridge port & the roscore port
                // TODO: the ROS master port
                portsString += " --publish=9090:9090 --publish=11311:11311 ";

                // sources ROS setup.bash
                string rosSetupString = " /bin/bash -c \"source /catkin_ws/devel/setup.bash && ";
                runCommandString += rosSetupString;
            }

            runCommandString += command;

            // if doing ros we need to add a " at the end
            if (setupROS == true) {
                runCommandString += "\"";
            }


            dockerRunString += $" {portsString} {dockerEnvString} {dockerVolString} {imageName} {runCommandString}";
            // dockerRunString += runCommandString;            


            string bashScript = $@"
{preDockerRunString}
{dockerRunString}                        
            ";

            UnityEngine.Debug.Log("INFO: Docker Run: " + $"-c \'{bashScript}\'");
            Task<int> t = ZOSystem.RunProcessAsync("/bin/bash", $"-c \'{bashScript}\'");

        }


        /// <summary>
        /// Docker Run with docker run parameters.
        /// </summary>
        /// <param name="runParameters"></param>
        public static void DockerRun(ZODockerRunParameters runParameters) {
            DockerRunROS(runParameters.imageName,
                        runParameters.containerName,
                        runParameters.command,
                        runParameters.volumes,
                        runParameters.ports,
                        runParameters.environments,
                        runParameters.runX11,
                        runParameters.setupROS,
                        runParameters.showOutput);
        }

        /// <summary>
        /// Stop a docker container.
        /// </summary>
        /// <param name="containerName">The name of the container.</param>
        public static void DockerStop(string containerName) {
            Task<int> t = ZOSystem.RunProcessAsync("docker", $"stop {containerName}");
        }




        /// <summary>
        /// Run a remote docker.
        /// <see>https://docs.docker.com/engine/reference/commandline/run/</see>
        /// </summary>
        /// <param name="uri"></param>
        /// <param name="imageName"></param>
        /// <param name="containerName"></param>
        /// <param name="command"></param>
        /// <param name="volumes"></param>
        /// <param name="ports"></param>
        /// <param name="environments">A list of environment variables to set inside the container.</param>
        /// <param name="setupROS">Do any ROS specific setup, such as opening up ports 9090 (ROS Bridge) and 11311 (ROS Master).</param>
        /// <param name="setupVNC">Do any VNC specific setup.</param>
        /// <param name="showOutput"></param>
        public static async Task DockerRunRemoteAsync(string uri,
                                                    string imageName,
                                                    string containerName,
                                                    string command,
                                                    List<VolumeMapEntry> volumes = null,
                                                    List<int> ports = null,
                                                    List<string> environments = null,
                                                    bool setupROS = false,
                                                    bool setupVNC = false,
                                                    bool showOutput = false) {


            // build the docker shared volumes
            Dictionary<string, EmptyStruct> configVolumes = new Dictionary<string, EmptyStruct>();
            if (volumes != null) {
                foreach (VolumeMapEntry v in volumes) {
                    configVolumes.Add(v.hostDirectory, new EmptyStruct());
                }
            }

            // build the docker --publish
            Dictionary<string, EmptyStruct> exposedPorts = new Dictionary<string, EmptyStruct>();
            Dictionary<string, IList<PortBinding>> portBindings = new Dictionary<string, IList<PortBinding>>();
            if (ports != null) {
                // setup exposed and bindings ports
                foreach (int port in ports) {
                    exposedPorts.Add($"{port}/tcp", new EmptyStruct());
                    portBindings.Add($"{port}/tcp", new List<PortBinding> {
                        new PortBinding {
                            HostIP="0.0.0.0", HostPort=$"{port}/tcp"
                        }
                    });
                }
            }


            // build the docker command.
            // NOTE:  The DockerClientConfiguration requires the command to be split into a list
            List<string> commmandList = new List<string>(command.Split(' '));

            // do ROS specific setup
            if (setupROS == true) {
                // add the default ros ports
                exposedPorts.Add("9090/tcp", new EmptyStruct());
                exposedPorts.Add("11311/tcp", new EmptyStruct());
                portBindings.Add("9090/tcp", new List<PortBinding> {
                    new PortBinding {
                        HostIP = "0.0.0.0",
                        HostPort = "9090"
                    }
                });
                portBindings.Add("11311/tcp", new List<PortBinding> {
                    new PortBinding {
                        HostIP = "0.0.0.0",
                        HostPort = "11311"
                    }
                });
            }

            try {
                using (DockerClient client = new DockerClientConfiguration(new Uri(uri)).CreateClient()) {
                    UnityEngine.Debug.Log("INFO: DockerRunRemote CreateContainerAsync");


                    var response = await client.Containers.CreateContainerAsync(new Docker.DotNet.Models.CreateContainerParameters {
                        Image = imageName,
                        Name = containerName,
                        Cmd = commmandList,
                        Volumes = configVolumes,
                        ExposedPorts = exposedPorts,
                        Env = environments,

                        // setup host config port bindings.
                        HostConfig = new HostConfig {
                            PortBindings = portBindings
                        },

                        // setup to get docker output
                        Tty = true,
                        AttachStderr = true,
                        AttachStdin = true,
                        AttachStdout = true

                    });

                    UnityEngine.Debug.Log("INFO: DockerRunRemote StartContainerAsync");
                    client.Containers.StartContainerAsync(response.ID, new Docker.DotNet.Models.ContainerStartParameters { }).Wait();

                    // Show the docker output
                    if (showOutput == true) {
                        await client.Containers.GetContainerLogsAsync(response.ID, new ContainerLogsParameters {
                            Follow = true,
                            ShowStderr = true,
                            ShowStdout = true
                        },
                                default(CancellationToken),
                                new Progress<string>(log => UnityEngine.Debug.Log($"INFO: <color=#4688f1>Docker: {log}</color>"))
                        );

                    }

                    UnityEngine.Debug.Log("INFO: DockerRunRemote End...");
                }

            } catch (System.Exception e) {
                UnityEngine.Debug.LogError("ERROR: DockerRunRemoteAsync Exception: " + e.ToString());
                throw;
            }
        }

        public static async Task DockerRunRemoteAsync(ZODockerRunParameters runParameters) {
            await DockerRunRemoteAsync(runParameters.remoteDockerUri,
                                        runParameters.imageName,
                                        runParameters.containerName,
                                        runParameters.command,
                                        runParameters.volumes,
                                        runParameters.ports,
                                        runParameters.environments,
                                        runParameters.setupROS,
                                        runParameters.setupVNC,
                                        runParameters.showOutput);
        }

        public static async Task DockerStopRemoteAsync(string containerName, string uri) {
            using (DockerClient client = new DockerClientConfiguration(new Uri(uri)).CreateClient()) {

                await client.Containers.StopContainerAsync(containerName, new ContainerStopParameters());
                await client.Containers.RemoveContainerAsync(containerName, new ContainerRemoveParameters());
            }
        }
    }
}