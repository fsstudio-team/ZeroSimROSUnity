using ZO.ROS.MessageTypes.ControllerManager;
namespace ZO.ROS.Controllers {
    public enum ControllerStateEnum {
        Initialized,
        Running
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

        void Load();
        void Unload();

    }
}