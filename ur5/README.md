## Project Core (`UR5`)

This directory (`UR5`) contains the core code for the project, including the UR5 description and the main logic. The code is organized into ROS 2 packages.

As described in the ROS 2 documentation, a package serves as an organizational unit for your code, enabling easier installation, sharing, and release.

Within this directory, you will find the following packages, each starting with `mycobot_` for consistency:

1.  **`mycobot_description`**: 
2.  **`mycobot_gazebo`**: 
3.  **`mycobot_gripper_description`**: 
4.  **`mycobot_moveit`**: 
5.  **`mycobot_mtc`**: 
6.  **`mycobot_opencv`**: 
7.  **`mycobot_ros2`**: 

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
