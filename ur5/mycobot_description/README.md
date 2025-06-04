# MyCobot Description Package (`mycobot_description`)

This ROS 2 package provides the URDF (Unified Robot Description Format) model for a MyCobot (based on a Universal Robots UR5 arm) equipped with a Robotiq 85 gripper. It includes descriptions for the robot and its workspace (table, storage units) and integrates with `ros2_control` for both simulation and real hardware.

## Key Features

*   **Robot Model:** Detailed URDF for the MyCobot UR5 arm and Robotiq 85 gripper, including visual and collision meshes.
*   **Configurable:** Uses XACRO for a highly configurable setup (e.g., simulation mode, real hardware IP, safety limits).
*   **`ros2_control` Ready:** Pre-configured for:
    *   Real robot control via `ur_robot_driver/URPositionHardwareInterface`.
    *   Gazebo Classic simulation.
    *   `mock_components/GenericSystem` for testing without hardware/full simulation.
*   **Workspace Elements:** Includes URDF for a table and storage units.
*   **Simulation Enhancements:**
    *   Gazebo Force-Torque sensor plugin on the wrist.
    *   Gazebo grasping plugin for the Robotiq gripper.
*   **Parameterization:** Robot characteristics (default kinematics, joint limits, physical properties) are loaded from YAML files in the `config/` directory.

## Package Structure

*   `urdf/`: Contains the core XACRO files defining the robot, gripper, and `ros2_control` interfaces.
    *   `ur_with_gripper.urdf.xacro` (or similar top-level file): Assembles the complete robot system. **This is often the main entry point.**
    *   `ur_robot.xacro`: Macro for the UR arm.
    *   `ur.ros2_control.xacro`: `ros2_control` system definition.
    *   `workspace/`: XACROs for workspace elements (table, storage).
*   `config/`: YAML files for robot parameters:
    *   `initial_positions.yaml`: Default joint states for simulation.
    *   `ur5/joint_limits.yaml`: Joint limits (position, velocity, effort).
    *   `ur5/default_kinematics.yaml`: Kinematic chain parameters.
    *   `ur5/physical_parameters.yaml`: Mass, inertia.
    *   `ur5/visual_parameteres.yaml`: Visual details.
*   `meshes/`: Contains STL and DAE files for visual and collision geometries.

## How to Use

This package is primarily used to load the `robot_description` parameter, which is then consumed by other ROS 2 nodes like `robot_state_publisher`, RViz, MoveIt 2, and Gazebo.

1.  **Configuration:**
    *   Modify YAML files in `config/` to suit your specific UR5 calibration, desired joint limits, or initial poses.
    *   Adjust XACRO arguments in your launch files (e.g., `sim_gazebo:=true`, `robot_ip:="192.168.X.X"`) to switch between simulation, fake hardware, or real robot control.

2.  **Launching:**
    A typical launch file would use the `xacro` command to process the main URDF XACRO file (e.g., `ur_with_gripper.urdf.xacro`) and load its output into the `robot_description` parameter.

    ```python
    # ...
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("mycobot_description"), "urdf", "ur_with_gripper.urdf.xacro"]
        ),
        " ",
        "name:=ur",
        " ",
        "ur_type:=ur5",
        " ",
        "sim_gazebo:=true", # or false
        " ",
        "use_fake_hardware:=false", # or true
        # ... other xacro arguments
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    # ... add other nodes like RViz, joint_state_publisher_gui, controllers
    ```

This package forms the foundation for simulating and controlling the MyCobot in ROS 2.

The code foundation of the robot and gripper description was built on top of Juo Tung Chen, ROS 2 pick and place project [https://github.com/JuoTungChen/ROS2_pick_and_place_UR5/tree/master].
