# ZeroSim 

ZeroSim is a robotics simulation engine built on the easy to use [Unity 3D](https://unity.com/)  development platform and the power of the [Robotics Operating System (ROS)](https://www.ros.org/).  ZeroSim is designed for ease of use and rapid development of all sorts of robotics and simulation -- from warehouses and industrial settings, to farming and outdoors -- from robotic arms to ground and drone based mobile robots.

ZeroSim is a project developed over several years by [FS Studios](https://fsstudio.com/?gclid=CjwKCAjw9MuCBhBUEiwAbDZ-7gpTTbBtgXtQe5VmZd_glTheBonWnaXt30lAFk5efc5mhaChyRNADBoC2EcQAvD_BwE) for the rapid development of all sorts of robotic simulation projects for various clients, from robotic arms to mobile robots.

We are releasing ZeroSim as open source to support the community of roboticist and software engineers that have supported us over the decades.  We are in active development and welcome all feature requests, bug reports, and pull requests.

![MoveIt Example](./Documentation~/images/zerosim_moveit.gif)
![Mobile Robot Example](Documentation~/images/zerosim_turtlebot_hospital.gif)
## Overview

ZeroSim provides a multitude of tools for building robots and environments in Unity to interface with ROS.  We strive to provide the same functionality and ROS interfaces of [Gazebo](http://gazebosim.org/).  Including:

* Dynamics simulation using the latest [PhysX 4.x](https://developer.nvidia.com/physx-sdk) integrated int Unity.
  * Hinge, ball, linear and fixed joints.
  * Temporal Gauss-Seidel solver option making articulated or jointed configurations much more robust.
* Advanced 3D Rendering, including the latest realtime ray tracing technology.
* Sensors:
  * 2D LIDAR -> ROS [LaserScan]([sensor_msgs/LaserScan.msg](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)) message.
  * 3D LIDAR -> ROS [LaserScan]([sensor_msgs/LaserScan.msg](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)) message.
  * Color camera -> ROS [Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) and ROS [CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) message.
  * Color + depth camera -> ROS [Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) and ROS [CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) message.
  * Stereo camera -> ROS [Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) and ROS [CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) message.
  * IMU -> ROS [Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) message
  * Magnetometer -> ROS [MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html) message.
  * Contact switch

* Ready to use ROS standard controllers and plugins:
  * Differential drive.  Controlled via standard ROS [Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) message.
  * Robotic arm controller via the ROS [FollowJointTrajectory Action Controller Interface](http://docs.ros.org/en/electric/api/control_msgs/html/msg/FollowJointTrajectoryAction.html) allowing [MoveIt](https://moveit.ros.org/) to connect seamlessly with ZeroSim.
  * ROS [TF](http://wiki.ros.org/tf) publisher.
  * ROS [JointState](http://wiki.ros.org/joint_state_publisher) publisher.

* Full ROS messaging communications layers API.
  * Premade standard ROS messages
  * Subscribe and publish
  * Action Servers
  * Fast and efficient message encoding using [ROS Bridge](http://wiki.ros.org/rosbridge_suite) and BSON over TCP.  

* Ready to run prebuilt ROS Docker images available publically on DockerHub: https://hub.docker.com/r/zerodog/zerosim_ros_vnc

* **COMING SOON:**
  * More complete documentation.
  * Secure communications via WebSockets.
  * URDF import & export.
  * Support for other Physics engines such as Bullet or Havok.

## Getting Started

### Recommended System

  * Ubuntu 18.04 or 20.04 (may work on MacOS or Windows but currently untested)
  * Unity 2020.x
  * ROS Melodic (ZeroSim provides a pre-built Docker container for ROS functionality https://hub.docker.com/r/zerodog/zerosim_ros_vnc)

### Setting up a new Unity Project

1. In Unity Hub create a new Unity project using Unity 2020.x or later. ![New Unity Project](Documentation~/images/new_unity_project.png)
2. Add ZeroSim via Unity Packages:  
   1. Unity Menu `Window -> Package Manager`
   2. Select the `+` dropdown:   
   ![Dropdown](Documentation~/images/unity_package_manager.png)
   1. Select `Add Package From Git URL...` and enter `git@github.com:fsstudio-team/ZeroSimROSUnity.git`.  Note this can take upto a few minutes to update but you should see the following:  
   ![ZeroSim Package Installed](Documentation~/images/zerosim_package_installed.png) 
   1. Import the ZeroSim Sample by selecting the Samples `Import` button in the Package Manager:  
    ![Import ZeroSim Samples](Documentation~/images/import_zerosim_samples.png)
3. If running Unity on Linux you want to avoid using OpenGL and use Vulkan, otherwise image based sensors may run slowly or not at all.  To change to using Vulkan:  
   1. In the Unity Menu: `Edit -> Project Settings...`:  
   2. Uncheck `Auto Graphics API for Linux` and then under `Graphics APIs for Linux` set `Vulkan` ahead of `OpenGL`:  
   ![Vulkan Settings](Documentation~/images/vulkan_settings.png) 


### Getting ZeroSim ROS Docker Container

1.  Available at https://hub.docker.com/r/zerodog/zerosim_ros_vnc or `docker pull zerodog/zerosim_ros_vnc:latest`
### Running TurtleBot Test Scene

*NOTE:* Order of operations is important.  Especially starting the Docker *before* the Unity simulation.

1. Make sure that the ZeroSim samples are installed as outlined above.
2. Make sure that the ZeroSim Docker container above is installed.
3. Open the `Scenes/Turtlebot3_Waffle_test.scene`
4. Launch the ZeroSim Docker via: 
```
docker run -it --rm \
--publish=9090:9090 \
--publish=11311:11311 \
--publish=8083:8083 \
--publish=80:80 \
--publish=5678:5678 \
--name my_zerosim_vnc_docker \
zerodog/zerosim_ros_vnc:latest \
roslaunch zero_sim_ros basic_unity_editor.launch

```
5. Run the ROS teleop in a seperate terminal by running: 
```
docker exec -it my_zerosim_vnc_docker \
bash -c "source devel/setup.bash ; rosrun turtlebot3_teleop turtlebot3_teleop_key"
```
6. In the Unity editor press the "Play" button.
7. The Turtlebot can now be controlled via the `w a s d` keys in the ROS teleop console window:

### Using RViz Example

This will show visualizing the 2D LIDAR and ROS TF in RViz.  This uses a VNC viewer to the ZeroSim ROS Docker container.

1. Startup the Turtlebot Test Scene as detailed above.
2. Open a noVNC connection by:
   1. In a browser open http://localhost:8083/vnc.html
   2. Press the "Connect" button. ![noVNC Login](Documentation~/images/novnc_login.png)
3. In the VNC window press the *LEFT* mouse button and select "Terminal". ![noVNC Terminal](Documentation~/images/novnc_terminal.png)
4. In the new terminal run `rviz -d ./src/zero_sim_ros/rviz/turtlebot_viewer.rviz`.  RViz will start up with a 3D view with the LIDAR scanner visibile. ![RViz Turtlebot Viewer](Documentation~/images/rviz_turtlebot.gif)