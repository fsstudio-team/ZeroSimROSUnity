using UnityEngine;
using UnityEditor;
using ZO.Util;
using ZO.ROS.Unity;

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
                    ZODocker.DockerRun(dockerRunParams);
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
