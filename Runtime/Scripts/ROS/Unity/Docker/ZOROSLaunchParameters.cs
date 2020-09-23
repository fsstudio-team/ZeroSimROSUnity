using System.Collections.ObjectModel;
using UnityEngine;
using System.Collections.Generic;

namespace ZO.ROS.Unity.Docker {
    [CreateAssetMenu(fileName = "ROSLaunch", menuName = "ZeroSim/ZOROSLaunchParameters", order = 1)]
    public class ZOROSLaunchParameters : ScriptableObject {

        /// <summary>
        /// The Docker service to use. For example "zosim".
        /// </summary>
        public string dockerServiceName = "zosim";
        
        /// <summary>
        /// ROS package name.  For example "zero_sim_ros".
        /// </summary>
        public string rosPackage = "zero_sim_ros";

        /// <summary>
        /// ROS launch file. For example "basic_unity_editor.launch"
        /// </summary>
        public string launchFile = "basic_unity_editor.launch";

        /// <summary>
        /// ROS launch arguments if any.
        /// </summary>
        /// <typeparam name="string"></typeparam>
        /// <returns></returns>
        public List<string> arguments = new List<string>();

        /// <summary>
        /// Volume maps for the docker.
        /// </summary>
        /// <returns></returns>
        public List<string> volumeMappings = new List<string>();
    }

}
