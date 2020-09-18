using System.Diagnostics;
using System.Threading.Tasks;
using System.Collections.Generic;
using System;

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
        /// Docker run 
        /// </summary>
        /// <param name="imageName"></param>
        /// <param name="command"></param>
        /// <param name="volumes"></param>
        /// <param name="ports"></param>
        /// <param name="environments"></param>
        /// <param name="runX11"></param>
        /// <param name="setupROS"></param>
        public static void DockerRun(string imageName,
                             string command,
                             List<VolumeMapEntry> volumes = null,
                             List<int> ports = null,
                             List<string> environments = null,
                             bool runX11 = false,
                             bool setupROS = false) {

            // build volumes string
            string volumeString = "";
            if (volumes != null) {
                foreach (VolumeMapEntry volumeMapEntry in volumes) {
                    volumeString += volumeMapEntry.String();
                }
            }

            string portsString = "";
            if (ports != null) {
                foreach (int port in ports) {
                    portsString += $" --publish={port}:{port}";
                }
            }

            string environmentString = "";
            if (environments != null) {
                foreach (string env in environments) {
                    environmentString += $" --env={env}";
                }
            }

            string dockerRunString = "run -it -rm ";

            // build up necessary stuff for X11
            if (runX11 == true) {
                environmentString += " --env=DISPLAY=$DISPLAY";
                environmentString += " --env=QT_X11_NO_MITSHM=1";
                environmentString += " --env=XAUTHORITY=$XAUTH";
                volumeString += " --volume=$XAUTH:$XAUTH";
                dockerRunString += " --gpus all";
                portsString += " --publish=9090:9090";
            }

            string runCommandString = "";

            if (setupROS == true) {
                // sources ROS setup.bash
                string rosSetupString = " /bin/bash -c \"source /catkin_ws/devel/setup.bash && ";
                runCommandString += rosSetupString;
            }

            runCommandString += command;

            dockerRunString += $" {imageName}";
            dockerRunString += runCommandString;


            // if doing ros we need to add a " at the end
            if (setupROS == true) {
                runCommandString += "\"";
            }

            // find the docker executable
            int exitCode = -99;
            string dockerExecutable = ZOSystem.RunProcessAndGetOutput("./", "which", "docker", out exitCode);

            UnityEngine.Debug.Log("INFO: Docker Run: " + dockerExecutable + " " + dockerRunString);
            Task<int> t = ZOSystem.RunProcessAsync(dockerExecutable, dockerRunString);

        }

        public static void DockerRun(ZODockerRunParameters runParameters) {
            DockerRun(runParameters.imageName, runParameters.command,
                        runParameters.volumes, runParameters.ports,
                        runParameters.environments,
                        runParameters.runX11,
                        runParameters.setupROS);
        }
    }
}