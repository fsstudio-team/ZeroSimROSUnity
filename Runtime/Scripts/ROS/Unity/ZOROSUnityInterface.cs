namespace ZO.ROS.Unity {

    /// <summary>
    /// Base class for objects that want to interface with ROS & ROS Bridge.
    /// </summary>
    public interface ZOROSUnityInterface {

        string ROSTopic { get; }
        void OnROSBridgeConnected(ZOROSUnityManager rosUnityManager);
        void OnROSBridgeDisconnected(ZOROSUnityManager rosUnityManager);
    }
}