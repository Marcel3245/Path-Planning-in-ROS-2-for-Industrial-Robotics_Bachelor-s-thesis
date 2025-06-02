
# Bachelor's thesis: Path Planning in ROS 2 for Industrial Robotics

An integrated ROS 2 system for automated, collision-free path planning for a Universal Robot 5 (UR5). Utilizes MoveIt for motion planning, Gazebo for simulation, and RViz for visualization. Incorporates OpenCV for basic object detection to dynamically update the planning scene from camera input, enabling more responsive path planning. Trajectories are validated in Gazebo to ensure safety and correctness, aiming to automate path generation and reduce manual programming in industrial robotics.
## Environment Setup: Installing WSL2 and Ubuntu 24.04

This project is built using the ROS 2 software framework, which requires a Linux environment to run. For users on Windows, the recommended and most straightforward approach is to install and use Windows Subsystem for Linux 2 (WSL2) with a compatible Ubuntu distribution. 

**Important:** This project specifically requires **Ubuntu 24.04 LTS (Noble Numbat)**.

Here's how to set up your environment:

1.  **Install WSL2:** If you don't already have WSL installed, follow the official Microsoft documentation. It's recommended to install WSL2 for better performance and compatibility.
    [Link to Microsoft WSL Installation Guide - e.g., https://learn.microsoft.com/en-us/windows/wsl/install]

2.  **Install Ubuntu 24.04 LTS:** Once WSL2 is set up, you can install Ubuntu 24.04 LTS.
    *   You can typically find "Ubuntu 24.04 LTS" in the Microsoft Store. 
    *   Alternatively, you can install it directly using the `wsl` command in PowerShell or Command Prompt:
        ```bash
        wsl --install -d Ubuntu-24.04
        ```
        This command installs WSL (if not already present) and sets Ubuntu 24.04 as the default distribution.

    For detailed instructions on installing Ubuntu 24.04 on WSL2, you can refer to the official Ubuntu documentation:
    [https://documentation.ubuntu.com/wsl/stable/howto/install-ubuntu-wsl2/]. 

After completing these steps, you will have a working Ubuntu 24.04 environment running within WSL2, ready for the next steps of setting up ROS 2 and the project dependencies.
## Installation: ROS 2 Jazzy and Gazebo Harmonic

This project utilizes the ROS 2 Jazzy distribution and the Gazebo Harmonic simulator. Follow the steps below to install them on your Ubuntu 24.04 environment within WSL2.

1.  **Access your Ubuntu 24.04 terminal:**
    Open PowerShell or Command Prompt in Windows and enter:
    ```bash
    wsl
    ```
    This will open a terminal session for your default WSL distribution, which should be Ubuntu 24.04 as set up in the previous steps.

2.  **Install ROS 2 Jazzy (Desktop Install):**
    Follow the official ROS 2 documentation for installing Jazzy on Ubuntu (Debian packages). It is highly recommended to perform the "Desktop Install" as it includes essential ROS tools, RViz, demos, and tutorials.

    Refer to the official guide here:
    [https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html]

3.  **Install Gazebo Harmonic:**
    Gazebo Harmonic is the recommended and most stable version of Gazebo to use with ROS 2 Jazzy on Ubuntu 24.04.

    Follow the official Gazebo documentation for installing Gazebo Harmonic on Ubuntu.

    Refer to the official guide here:
    [https://gazebosim.org/docs/harmonic/install_ubuntu/]

    Alternatively, when installing ROS 2 Jazzy, you can often install Gazebo Harmonic via the `ros-jazzy-ros-gz` package, which pulls the compatible Gazebo version from the ROS repositories.


4. **Install MoveIt 2**

    Ensure you have the necessary MoveIt 2 tools installed. The recommended approach is to follow the official binary installation guide: [https://moveit.ai/install-moveit2/binary/]

    For a quick installation, you can use the commands below. The second command installs a wider set of tools, including common motion planners like OMPL.

    ```bash
    # Install core MoveIt 2 packages
    sudo apt install ros-$ROS_DISTRO-moveit

    # Install additional MoveIt 2 tools and planners
    sudo apt install ros-$ROS_DISTRO-moveit*
    ```
## Prepare the Workspace

To set up and prepare the ROS 2 workspace for this project, follow these steps:

1.  **Configure your `.bashrc` file:**
    The `.bashrc` file is a script that runs every time you start a new terminal session. Editing it allows us to add helpful aliases and ensure the ROS 2 environment is sourced correctly. This makes working with the workspace more convenient.

    Add the following lines to your `~/.bashrc` file. You can do this by copying and pasting the block below into your terminal. This uses `cat` to append the lines to the file.

    ```bash
    cat >> ~/.bashrc << EOF
    # Custom aliases for ROS 2 workspace
    export _colcon_cd_root=/opt/ros/jazzy/
    alias build='cd ~/ros2_ws/ && colcon build --symlink-install && source install/setup.bash'
    # Useful for hard reset of src. Important: remember to build the workspace after! 
    alias reset_ws='cd ~/ros2_ws/ && rm -r /install /build /log && unset AMENT_PREFIX_PATH && unset CMAKE_PREFIX_PATH && source /opt/ros/jazzy/setup.bash'

    # Source the ROS 2 setup file
    source /opt/ros/jazzy/setup.bash

    # Fix for Gazebo GUI permissions in some WSL setups
    chmod 0700 /run/user/1000
    EOF
    ```

    **Explanation of Aliases:**
    *   `build`: Navigates to the workspace root, builds the packages using `colcon build`, and then sources the `install/setup.bash` file to make the built packages available in the current terminal session. Added `--symlink-install` for faster builds during development.
    *   `reset_ws`: Unsets workspace-specific environment variables and re-sources the base ROS 2 Jazzy environment. Useful for starting fresh or switching between workspaces.
    *   `backup_ws`: Creates a timestamped backup of your `src` directory in a `~/backups` folder.
    *   `reload_ws`: Cleans the workspace (removes `build`, `install`, `log`, `src`), recreates the `src` directory, and copies the *most recent* backup of your `src` into it.
    *   `save_ws`: Removes the *most recent* backup and creates a new timestamped backup of your current `src` directory. (Note: The logic here is slightly different from your original `save_ws` which seemed to remove the latest and *then* create a new one based on the *new* count, which could be confusing. This version removes the latest and then creates a *new* timestamped one.)

    After adding the aliases, either close and reopen your terminal or run `source ~/.bashrc` for the changes to take effect in your current session.

2.  **Create and Populate the Workspace:**
    Now, create the workspace directory structure, clone the project code into the `src` folder, and perform the initial build.

    ```bash
    # Ensure git is installed
    sudo apt-get update && sudo apt-get install -y git

    # Create workspace directories
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src

    # Clone the project code into the src directory
    git clone https://github.com/Marcel3245/Path-Planning-in-ROS-2-for-Industrial-Robotics_Bachelor-s-thesis.git .

    # Install the dependencies
    cd ~/ros2_ws/
    rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO

    # Build the workspace (using the alias defined above)
    build
    ```

    The `build` command will compile the packages in your `src` directory. Once it completes successfully, your workspace is set up and ready.

## Run the Program

    To launch the complete system, use the following ROS 2 command from your workspace root:

    ```bash
    cd ~/ros2_ws/
    
    ros2 launch mycobot_mtc run.launch.py
    ```

**Upon execution, three windows should open:**

    1. Gazebo: The simulation environment showing the UR5 and workspace.
    2. RViz: The 3D visualization interface for monitoring the robot and planning scene.
    3. OpenCV Camera View: A window displaying the processed camera feed.

**Within the RViz window, you will find two interactive buttons:**

    1. Spawn Workpieces: Click this button to add six green workpiece objects into the designated storage area within the simulation.
    2. Run MTC: Press this button to initiate the MoveIt Task Constructor (MTC) process. MTC will search for possible collision-free trajectories to move a workpiece.
    
The system is configured to find a specific number of trajectories (as defined in the code).
If the MTC search is successful, the UR5 manipulator will automatically execute the trajectory determined to be the "cheapest" based on predefined cost metrics (also configured in the code). You can observe the robot's movement and the simulated workpiece transfer from the start to the target storage within the Gazebo simulation window, reflecting the planned real-world behavior.
