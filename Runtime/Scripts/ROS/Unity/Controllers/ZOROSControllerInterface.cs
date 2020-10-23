using ZO.ROS.MessageTypes.ControllerManager;
namespace ZO.ROS.Controllers {
    public enum ControllerStateEnum {
        Uninitialized,
        Stopped,
        Initialized,
        Running,
        Terminated
    };


    /// <summary>
    /// Defines a standard interface to ROS controllers from the controller manager.
    /// </summary>
    public interface ZOROSControllerInterface {
        string ControllerName { get; }
        string ControllerType { get; }

        string HardwareInterface { get; }

        ControllerStateEnum ControllerState { get; }

        ControllerStateMessage ControllerStateMessage { get; }

        /// <summary>
        /// Set controller state to loaded/initialized
        /// </summary>
        void LoadController();

        /// <summary>
        /// Set controller state to unloaded/stopped
        /// </summary>
        void UnloadController();

        /// <summary>
        /// Set controller state to started/running
        /// </summary>
        void StartController();

        /// <summary>
        /// Set controller state to stopped
        /// </summary>
        void StopController();

    }
}