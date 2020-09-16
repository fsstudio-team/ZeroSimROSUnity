using UnityEngine;
using UnityEditor;
using ZO.ROS.Unity;
using ZO.ROS.Unity.Docker;

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
                // if we are entering play mode then launch ROS 
                ZOROSLaunchParameters launchParameters = ZOROSUnityManager.Instance?.ROSLaunchParameters;

                if (launchParameters) {
                    string ros_setup = "/bin/bash -c \"source /catkin_ws/devel/setup.bash && ";
                    string command = ros_setup + $"roslaunch {launchParameters.rosPackage} {launchParameters.launchFile}\"";

                    ZODockerManager.DockerRun(launchParameters.dockerServiceName,
                                            command,
                                            launchParameters.volumeMappings.ToArray(),
                                            true, true,
                                            (exitCode) => {

                                                if (exitCode != 0) {
                                                    UnityEngine.Debug.LogError($"Docker command error exit code: {exitCode}");
                                                    return;
                                                }

                                                UnityEngine.Debug.Log($"Docker command exit code: {exitCode}");
                                            });
                }
            } else if (state == PlayModeStateChange.ExitingPlayMode) {
                ZODockerManager.DockerComposeDown();
            }
        }
    }
}
