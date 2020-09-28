using UnityEngine;
using UnityEditor;
using ZO.Util;
using ZO.ROS.Unity;
using System.Threading.Tasks;

namespace ZO.Editor {

    /// <summary>
    /// Monitors if user starts application in the editor. 
    /// </summary>
    [InitializeOnLoadAttribute]
    public static class PlayModeStateChangedExample {
        // register an event handler when the class is initialized
        static PlayModeStateChangedExample() {
            EditorApplication.playModeStateChanged += LogPlayModeState;
        }

        private static void LogPlayModeState(PlayModeStateChange state) {
            Debug.Log("INFO: Editor State Change: " + state.ToString());
            if (state == PlayModeStateChange.EnteredPlayMode) {
                // if we are entering play mode then launch ROS docker
                ZODockerRunParameters dockerRunParams = ZOROSUnityManager.Instance?.ROSLaunchParameters;

                if (dockerRunParams) {
                    if (dockerRunParams.runRemoteDocker == true) {
                        // Use the Docker Remote API to launch a remote docker.
                        var t = Task.Run(async () => {

                            await ZODocker.DockerRunRemoteAsync(dockerRunParams);

                            // await ZODocker.DockerRunRemoteAsync("unix:///var/run/docker.sock",
                            //                     "docker.pkg.github.com/fsstudio-team/zerosimros/zerosim_ros",
                            //                     "/bin/bash -c \"source /catkin_ws/devel/setup.bash && roslaunch zero_sim_ros basic_unity_editor.launch\"", null, null, null, true, false);

                        });

                    } else { // run local docker
                        ZODocker.DockerRun(dockerRunParams);
                    }
                    
                }
            } else if (state == PlayModeStateChange.ExitingPlayMode) {
                // if we are exiting play mode stop docker
                ZODockerRunParameters dockerRunParams = ZOROSUnityManager.Instance?.ROSLaunchParameters;
                if (dockerRunParams) {
                    ZODocker.DockerStop(dockerRunParams.containerName);
                }

                // ZODockerManager.DockerComposeDown();
                // TODO: stop docker
            }
        }
    }
}
