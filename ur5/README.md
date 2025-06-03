## Project Core (`UR5`)

This directory (`UR5`) contains the core code for the project, including the UR5 description and the main logic. The code is organized into ROS 2 packages.

As described in the ROS 2 documentation, a package serves as an organizational unit for your code, enabling easier installation, sharing, and release.

Within this directory, you will find the following packages, each starting with `mycobot_` for consistency:

1.  **`mycobot_description`**: is a package which holds specific information about UR5, from its colision and visual description, to its kinematics parameters and limits.
```
      | --- config -> (kinemtaics, limits, etc of the robot)
      | --- meshes -> (visual and colision description)
      \ --- urdf -> (combines and translates the robot information into robotic overall description)
```
2.  **`mycobot_gazebo`**: is a package which defines gazebo simulation.
```
      | --- config -> (describes the bridge communication between ros and gazebo)
      | --- models -> (stores visual, colision, and parameter descriptions of the parts which are only in gazebo simulation)
      \ --- worlds -> (describes the simulation world in gazebo: GUI, simulation parts in gz, world parameters, such as gravity or magnetic force, etc)
```
3.  **`mycobot_gripper_description`**: is a package which holds specific information about robotiq gripper, its colision and visual description.
```
      | --- meshes -> (visual and colision description)
      \ --- urdf -> (combines and translates the gripper information into robotic overall description)
```
4.  **`mycobot_moveit`**: it provides a comprehensive framework for motion planning, manipulation, 3D perception, kinematics, control, and navigation. Essentially, it allows robots to plan and execute movements to interact with their environment. 
```
      | --- config -> (important data for MoveIt path planning and robot configuration)
      | --- srdf -> (Semantic Robot Description Format. It complements the URDF file)
      \ --- .setup_assistant -> (helps to MoveIt to define final robot description)
```
5.  **`mycobot_mtc`**: define and plan complex robot tasks involving multiple, interdependent subtasks.
```
      | --- launch -> (stores the possible launch configurations of the program)
      | --- rviz -> (describes the rviz GUI)
      \ --- src -> (directory for source code files)
```
6.  **`mycobot_opencv`**: define the configuration of object detection, using OpenCV library
```
      \ --- mycobot_opencv -> (run the opencv/camera node) 
```
7.  **`mycobot_ros2`**: it is the fundamental unit of organization for your code. It's essentially a folder containing files related to a specific functionality or component of a robot system.   
      
  
Each of these packages typically follows a standard ROS 2 structure, including these main files and directories:

*   **`CMakeLists.txt`**: Defines how to build the code within the package.
*   **`package.xml`**: Contains meta-information about the package (dependencies, author, etc.).
*   **`include/<package_name>`**: Directory for public header files.
*   **`src`**: Directory for source code files (.cpp, .py).

A common package structure looks like this:
```
my_package/
      | --- include/my_package/
      | --- src/
      | --- package.xml
      \ --- CMakeLists.txt
```

For a more detailed understanding of ROS 2 packages, refer to the official documentation: [https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html]
