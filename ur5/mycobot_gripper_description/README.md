# MyCobot Gripper Description (`mycobot_gripper_description`)

This ROS 2 package provides the URDF (Unified Robot Description Format) XACRO macros for a Robotiq 85 parallel gripper. It defines the gripper's links, joints, visual meshes, collision geometries, and inertial properties, making it suitable for simulation and integration with robot arms.

## Key Features

*   **Detailed Gripper Model:** Provides a complete URDF model of the Robotiq 85 gripper.
*   **XACRO Macro:** Defines a `robotiq_85_gripper` XACRO macro for easy instantiation and attachment to a parent link (e.g., a robot arm's end-effector).
    *   Parameters: `prefix`, `parent` (link), `*origin` (transform block).
*   **Kinematics:**
    *   Includes all necessary links: `robotiq_85_base_link`, `..._knuckle_link`, `..._finger_link`, `..._inner_knuckle_link`, `..._finger_tip_link`.
    *   Utilizes `mimic` joints for synchronized movement of the gripper fingers, driven by the `robotiq_85_left_knuckle_joint`.
*   **Visuals and Collisions:** Uses DAE (visual) and STL (collision) meshes located in the `meshes/` directory for realistic appearance and physics interaction.
*   **Gazebo Simulation Enhancements:**
    *   Includes `<gazebo>` tags with specific physics properties (`<kp>`, `<kd>`, `<mu1>`, `<mu2>`, `<minDepth>`) for the `..._finger_tip_link`s to improve grasping behavior in Gazebo.

## Package Structure

*   `urdf/`:
    *   **`robotiq_85_gripper.urdf.xacro`**: This is the **core file** defining the `robotiq_85_gripper` XACRO macro with all its links, joints, and properties.
    *   `robotiq_85_gripper.transmission.xacro`: Defines `<transmission>` elements for the gripper's actuated joint.
    *   `robotiq_85_gripper.xacro`: A top-level example URDF that instantiates the gripper attached to a simple `gripper_root_link` for standalone viewing or testing. 
    *   `robotiq_85_gripper_sim_base.xacro`: A utility macro to create a simple base link for the gripper in simulation.
*   `meshes/`:
    *   `visual/`: Contains DAE mesh files for visualization.
    *   `collision/`: Contains STL mesh files for collision checking.

## How to Use

This package is primarily intended to be used as a component within a larger robot's URDF.

1.  **Include in your Robot's XACRO:**
    Add an include statement for `robotiq_85_gripper.urdf.xacro` and then call the macro.

    ```xml
    <xacro:include filename="$(find mycobot_gripper_description)/urdf/robotiq_85_gripper.urdf.xacro" />

    <!-- Attach the robotiq 85 gripper -->
    <xacro:robotiq_85_gripper prefix="your_prefix_" parent="your_robot_tool_link">
      <origin xyz="0 0 0.01" rpy="0 0 0"/>
    </xacro:robotiq_85_gripper>
    ```

2.  **`ros2_control` Integration:**
    When using this gripper with `ros2_control`, the command and state interfaces for its joints (e.g., `your_prefix_robotiq_85_left_knuckle_joint`) should be defined in the `<ros2_control>` section of your main robot's URDF. The `mycobot_description` package provides an example of this. This `mycobot_gripper_description` package provides the mechanical and visual model, while the control interfaces are typically managed by the integrating robot's `ros2_control` setup.

## Simulation

*   The finger tip links have Gazebo-specific tags to enhance grasping physics.
*   The example `urdf/robotiq_85_gripper.xacro` can be used to spawn the gripper standalone in Gazebo. However, for ROS 2, you would typically use `ros2_control` and the `gz_ros2_control` (Gazebo) or `ign_ros2_control` (Ignition) plugins, which are configured in the robot system that *includes* this gripper.
