# ZeroSim
- [ZeroSim](#zerosim)
  - [Overview](#overview)
  - [API Documentation](#api-documentation)
  - [Getting Started](#getting-started)
    - [Video Tutorials](#video-tutorials)
    - [Recommended System](#recommended-system)
    - [Setting up a new Unity Project](#setting-up-a-new-unity-project)
    - [Getting ZeroSim ROS Docker Container](#getting-zerosim-ros-docker-container)
    - [Running TurtleBot Test Scene](#running-turtlebot-test-scene)
    - [Using RViz for Turtlebot](#using-rviz-for-turtlebot)
    - [Running Universal Robot UR10 Arm Test Scene with MoveIt!](#running-universal-robot-ur10-arm-test-scene-with-moveit)
    - [Running Image Segmentation Test](#running-image-segmentation-test)
    - [Export URDF](#export-urdf)
    - [Import URDF](#import-urdf)

ZeroSim is a robotics simulation engine built on the easy to use [Unity 3D](https://unity.com/)  development platform and the power of the [Robotics Operating System (ROS)](https://www.ros.org/).  ZeroSim is designed for ease of use and rapid development of all sorts of robotics and simulation -- from warehouses and industrial settings, to farming and outdoors -- from robotic arms to ground and drone based mobile robots.

ZeroSim is a project developed over several years by [FS Studios](https://fsstudio.com/?gclid=CjwKCAjw9MuCBhBUEiwAbDZ-7gpTTbBtgXtQe5VmZd_glTheBonWnaXt30lAFk5efc5mhaChyRNADBoC2EcQAvD_BwE) for the rapid development of all sorts of robotic simulation projects for various clients, from robotic arms to mobile robots.

We are releasing ZeroSim as open source to support the community of roboticist and software engineers that have supported us over the decades.  We are in active development and welcome all feature requests, bug reports, and pull requests.

![MoveIt Example](./Documentation~/images/zerosim_moveit.gif)
![Mobile Robot Example](Documentation~/images/zerosim_turtlebot_hospital.gif)
## Overview

ZeroSim provides a multitude of tools for building robots and environments in Unity to interface with ROS.  We strive to provide the same functionality and ROS interfaces of [Gazebo](http://gazebosim.org/).  Including:

* Dynamics simulation using the latest [PhysX 4.x](https://developer.nvidia.com/physx-sdk) integrated in Unity.
  * Hinge, ball, linear and fixed joints.
* Advanced 3D Rendering, including the latest realtime ray tracing technology.
* Sensors:
  * 2D LIDAR -> ROS [LaserScan]([sensor_msgs/LaserScan.msg](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)) message.
  * 3D LIDAR -> ROS [PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)
  * Color camera -> ROS [Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) and ROS [CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) message.
  * Color + depth camera -> ROS [Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) and ROS [CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) message.
  * Stereo camera -> ROS [Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) and ROS [CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) message.
  * IMU -> ROS [Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) message
  * Magnetometer -> ROS [MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html) message.
  * Contact switch
  * Altimeter

* Ready to use ROS standard controllers and plugins:
  * Differential drive.  Controlled via standard ROS [Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) message.
  * Robotic arm controller via the ROS [FollowJointTrajectory Action Controller Interface](http://docs.ros.org/en/electric/api/control_msgs/html/msg/FollowJointTrajectoryAction.html) allowing [MoveIt](https://moveit.ros.org/) to connect seamlessly with ZeroSim.
  * ROS [TF](http://wiki.ros.org/tf) publisher.
  * ROS [JointState](http://wiki.ros.org/joint_state_publisher) publisher.

* Full ROS messaging communications layers API.
  * Many premade standard ROS messages
  * Subscribe and publish
  * Action Servers
  * Fast and efficient message encoding using [ROS Bridge](http://wiki.ros.org/rosbridge_suite) and BSON over TCP.
  * Parameter server  

* Ready to run prebuilt ROS Docker images available publically on DockerHub: https://hub.docker.com/r/zerodog/zerosim_ros_vnc

* Machine Learning tools:
  * Image Segmentation for training semantic segmentation algorithms. 

* URDF Import & Export 
* ROS2 support via ROS Bridge Suite (https://github.com/RobotWebTools/rosbridge_suite)

* **COMING SOON:**
  * Configure and run Docker images within Unity Editor.
  * Secure communications via WebSockets.
  * Drone controller.

## API Documentation

[API Documentation](https://fsstudio-team.github.io/ZeroSimROSUnity/api/index.html)

## Getting Started

### Video Tutorials

[![Tutorial #1: Install ZeroSim in Unity](./Documentation~/images/tutorial_1.png)](http://www.youtube.com/watch?v=Q_CyYt-9kY4&list=PL-GMA_Bq2CR9JnLjbMSXAtcpZFqwlu6mz "Video Title")

[![Tutorial #2: Create a New ZeroSim Unity Scene](./Documentation~/images/tutorial_2.png)](http://www.youtube.com/watch?v=dQXihVeI5YE&list=PL-GMA_Bq2CR9JnLjbMSXAtcpZFqwlu6mz "Video Title")

[![Tutorial #3: Connect to ROS](./Documentation~/images/tutorial_3.png)](http://www.youtube.com/watch?v=LvtnMmcixrY&list=PL-GMA_Bq2CR9JnLjbMSXAtcpZFqwlu6mz "Video Title")

[![Tutorial #4: Build a Robot From Scratch](./Documentation~/images/tutorial_4.png)](http://www.youtube.com/watch?v=xGjx8cUVctM&list=PL-GMA_Bq2CR9JnLjbMSXAtcpZFqwlu6mz "Video Title")

### Recommended System

  * Ubuntu 18.04 or 20.04 (may work on MacOS or Windows but currently untested)
  * Unity 2020.x or greater
  * ROS Melodic (ZeroSim provides a pre-built Docker container for ROS functionality https://hub.docker.com/r/zerodog/zerosim_ros_vnc)
    * Note: ROS Melodic is our primary supported development environment, but ZeroSim has reportedly been able to run on Noetic and ROS2.

### Setting up a new Unity Project 

1. In Unity Hub create a new Unity project using Unity 2020.x or later. ![New Unity Project](Documentation~/images/new_unity_project.png)
2. Add ZeroSim via Unity Packages:  
   1. Unity Menu `Window -> Package Manager`
   2. Select the `+` dropdown:   
   ![Dropdown](Documentation~/images/unity_package_manager.png)
   1. Select `Add Package From Git URL...` and enter `git@github.com:fsstudio-team/ZeroSimROSUnity.git`.  Note this can take upto a few minutes to update but you should see the following:  
   ![ZeroSim Package Installed](Documentation~/images/zerosim_package_installed.png) 
   1. Import the ZeroSim Samples by selecting the Samples `Import` button in the Package Manager:  
    ![Import ZeroSim Samples](Documentation~/images/import_zerosim_samples.png)
3. **IMPORTANT** the default Unity physics settings do not work well with a lot of simulations.  It is very much recommended (required for probably most all simulations) to set the physics settings by opening the Unity menu `Edit -> Project Settings... -> Physics` and set the `Default Solver Iterations` to `30` and the `Default Solver Velocity Iterations` to `60`. ![Unity Physics Settings](Documentation~/images/unity_physics_settings.png) 
4. **IMPORTANT** the default Unity fixed timestep setting does not work well with a lot of simulions.  It is very much recommended to set the `Fixed Timestep` setting by `Edit -> Project Settings... -> Time` to `0.005` ![Timestep Settings](Documentation~/images/unity_timestep_settings.png)
5. If running Unity on Linux you want to avoid using OpenGL and use Vulkan, otherwise image based sensors may run slowly or not at all.  To change to using Vulkan:  
   1. In the Unity Menu: `Edit -> Project Settings...`:  
   2. Uncheck `Auto Graphics API for Linux` and then under `Graphics APIs for Linux` set `Vulkan` ahead of `OpenGL`:  
   ![Vulkan Settings](Documentation~/images/vulkan_settings.png) 


### Getting ZeroSim ROS Docker Container

1.  Available at https://hub.docker.com/r/zerodog/zerosim_ros_vnc or `docker pull zerodog/zerosim_ros_vnc:latest`
### Running TurtleBot Test Scene

*NOTE:* Order of operations is important.  Especially starting the Docker *before* the Unity simulation.

1. Make sure that the ZeroSim samples are installed as outlined above.
2. Make sure that the ZeroSim Docker container above is installed.
3. Open the `Turtlebot3_Waffle_test.scene` ![Open Turtlebot Scene](Documentation~/images/open_turtlebot_scene.png)
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

### Using RViz for Turtlebot

This will show visualizing the 2D LIDAR and ROS TF in RViz.  This uses a VNC viewer to the ZeroSim ROS Docker container.

1. Startup the Turtlebot Test Scene as detailed above.
2. Open a noVNC connection by:
   1. In a browser open http://localhost:8083/vnc.html
   2. Press the "Connect" button. ![noVNC Login](Documentation~/images/novnc_login.png)
3. In the VNC window press the *LEFT* mouse button and select "Terminal". ![noVNC Terminal](Documentation~/images/novnc_terminal.png)
4. In the new terminal run `rviz -d ./src/zero_sim_ros/rviz/turtlebot_viewer.rviz`.  RViz will start up with a 3D view with the LIDAR scanner visibile. ![RViz Turtlebot Viewer](Documentation~/images/rviz_turtlebot.gif)


### Running Universal Robot UR10 Arm Test Scene with MoveIt!

*NOTE:* Order of operations is important.  Especially starting the Docker *before* the Unity simulation.

1. Make sure that the ZeroSim samples are installed as outlined above.
2. Make sure that the ZeroSim Docker container above is installed.
3. Open the `UniversalRobot_UR10_test.scene` ![Open UR10 Scene](Documentation~/images/ur10_test_scene.png)
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
roslaunch zero_sim_ros ur10_moveit.launch
```
5. Start the UR10 test Scene by pressing the Play button.
6. Open a noVNC connection by:
   1. In a browser open http://localhost:8083/vnc.html
   2. Press the "Connect" button. ![noVNC Login](Documentation~/images/novnc_login.png)
7. In the noVNC window RViz will be setup with MoveIt ![UR10 RViz MoveIt](Documentation~/images/ur10_moveit_rviz.png)
8. You can now to standard MoveIt! operations in RViz to control the UR10 arm ![UR10 MoveIt](Documentation~/images/ur10_moveit_rviz.gif)

### Running Image Segmentation Test

1. Make sure that the ZeroSim samples are installed as outlined above.
2. Make sure that the ZeroSim Docker container above is installed.
3. Open the `ImageSegmentation_test.scene` ![Open Segmentation Test Scene](Documentation~/images/image_segmentation_scene.png)
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
5. Start the Image Segmentation Test Scene by pressing the Play button.
6. Open a noVNC connection by:
   1. In a browser open http://localhost:8083/vnc.html
   2. Press the "Connect" button. ![noVNC Login](Documentation~/images/novnc_login.png)
7. In the VNC window press the *LEFT* mouse button and select "Terminal". ![noVNC Terminal](Documentation~/images/novnc_terminal.png)
8. In the new terminal run `rqt_image_view /image/segmentation_image`.  
9. Open up a second terminal and run `rqt_image_view /image/image_raw` ![RQT Image View Segmentation](Documentation~/images/rqt_image_view_segmentation.png)


### Export URDF

1. Make sure that the ZeroSim samples are installed as outlined above.
2. Open scene the `URDF_test.scene` in the ZeroSim samples.
3. Select `SimpleRobotArm` in the scene hierarchy.   
![Select Simple Robot Arm](Documentation~/images/select_simple_robot_arm.png)
4. Select `Export URDF` in the root properties view. ![Export URDF](Documentation~/images/export_urdf.png)
5. Select the directory to export to.
6. An excellent online URDF viewer is available: https://gkjohnson.github.io/urdf-loaders/javascript/example/index.html  Just drag and drop the files exported above.

### Import URDF

1. Right click and select `ZeroSim --> Import URDF...`
