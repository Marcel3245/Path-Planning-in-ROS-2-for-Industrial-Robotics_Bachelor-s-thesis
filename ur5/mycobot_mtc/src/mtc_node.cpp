#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h> // Use planning_scene_interface.h for add/remove collision objects
#include <geometric_shapes/shape_operations.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h> // Needed for tf2::Quaternion


#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp" // Include for Bool message
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <memory>
#include <math.h>
#include <mutex> 
#include <array>
using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mycobot_mtc");
namespace mtc = moveit::task_constructor;

// Global array to store circle data (assuming this is still needed for other parts)
float data_array[6][4] = {0}; // Likely stores circles data
std::mutex data_mutex_;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();

  // Subscription callbacks
  // Renamed to circles_data_callback for clarity, handling the original "camera/circles_to_world"
  void circles_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  // New callback specifically for the storage data
  void storage_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void spawn_workpieces_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void run_mtc_callback(const std_msgs::msg::Bool::SharedPtr msg);

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr circles_data_sub_; // Renamed
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr camera_data_storage_sub_; // Stays the same topic name

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spawn_workpieces_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_mtc_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // State variables
  bool workpieces_spawned_ = false;
  bool task_planned_executed_ = false;
  bool circles_data_received_ = false; // Flag for circles data
  bool storage_pose_data_received_ = false; // New flag for storage pose data
  bool target_storage_exists_ = false;

  // Class members to store storage pose data derived from camera
  float x_mid_storage = 0.0;
  float y_mid_storage = 0.0;
  float z_storage = 0.0; // Initialize to 0.0, will be set by callback
  float alfa = 0.0;      // Yaw angle for storage orientation

  geometry_msgs::msg::PoseStamped target_place_pose_; // Used for object placement, not storage spawn

  // ... other members (TF, start_storage_pose, etc.) ...
  std::optional<geometry_msgs::msg::Pose> get_start_storage_pose();
  std::optional<geometry_msgs::msg::Pose> start_storage_pose_; 

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  // Camera data subscription (circles) - Renamed callback
  circles_data_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "camera/circles_to_world", 10, std::bind(&MTCTaskNode::circles_data_callback, this, std::placeholders::_1));

  // Camera data subscription (storage) - New callback
  camera_data_storage_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "camera/storage_to_world", 10, std::bind(&MTCTaskNode::storage_data_callback, this, std::placeholders::_1)); // Use new callback

  // Button subscriptions
  spawn_workpieces_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "button1", 10, std::bind(&MTCTaskNode::spawn_workpieces_callback, this, std::placeholders::_1));

  run_mtc_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "button2", 10, std::bind(&MTCTaskNode::run_mtc_callback, this, std::placeholders::_1));

  // Publisher for visualization markers
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "mtc_node/visualization_marker_array", 10);

  target_place_pose_.header.frame_id = "world";
  target_place_pose_.pose.orientation.w = 1.0;

  // Initialize TF components
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // TF lookup for start_storage pose (used for adding small objects relative to TF,
  // distinct from the main storage mesh added from camera data)
  start_storage_pose_ = get_start_storage_pose();


  RCLCPP_INFO(LOGGER, "MTCTaskNode initialized. Waiting for button presses and camera data...");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

// Renamed from camera_data_callback
void MTCTaskNode::circles_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Existing logic to handle circle data and visualize markers
    const size_t expected_elements = 6 * 4;
    if (msg->data.size() < expected_elements) {
        RCLCPP_WARN(LOGGER, "Received circles data array is smaller than expected (got %zu, expected %zu elements)", msg->data.size(), expected_elements);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < 6; ++i) {
            for (size_t j = 0; j < 4; ++j) {
                size_t index = i * 4 + j;
                if (index < msg->data.size()) {
                    data_array[i][j] = msg->data[index];
                } else {
                     RCLCPP_ERROR(LOGGER, "Unexpected index out of bounds during circle data copy.");
                     break;
                }
            }
        }
        circles_data_received_ = true; // Indicate circles data received
    } // End mutex scope

    // Marker visualization remains the same (for circles)
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.resize(6);

    for (size_t i = 0; i < 6; ++i) {
        marker_array.markers[i].header.frame_id = "world";
        marker_array.markers[i].header.stamp = node_->now();
        marker_array.markers[i].ns = "circles_" + std::to_string(i);
        marker_array.markers[i].id = i;
        marker_array.markers[i].type = visualization_msgs::msg::Marker::CYLINDER;
        marker_array.markers[i].action = visualization_msgs::msg::Marker::ADD;
        marker_array.markers[i].pose.position.x = data_array[i][0];
        marker_array.markers[i].pose.position.y = data_array[i][1];
        marker_array.markers[i].pose.position.z = data_array[i][2];
        marker_array.markers[i].pose.orientation.x = 0.0;
        marker_array.markers[i].pose.orientation.y = 0.0;
        marker_array.markers[i].pose.orientation.z = 0.0;
        marker_array.markers[i].pose.orientation.w = 1.0;
        marker_array.markers[i].scale.x = 0.043; // Diameter
        marker_array.markers[i].scale.y = 0.043; // Diameter
        marker_array.markers[i].scale.z = 0.001; // Height
        if (data_array[i][3] == 0){
            marker_array.markers[i].color.r = 0.0f;
            marker_array.markers[i].color.g = 1.0f;
            marker_array.markers[i].color.b = 0.0f;
        } else {
            marker_array.markers[i].color.r = 1.0f;
            marker_array.markers[i].color.g = 0.0f;
            marker_array.markers[i].color.b = 0.0f;
        }
        marker_array.markers[i].color.a = 1.0f;
        marker_array.markers[i].lifetime = rclcpp::Duration(0, 0);
    }

    marker_pub_->publish(marker_array);
}

void MTCTaskNode::storage_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Expected data format: [x, y, z, yaw]
    if (msg->data.size() < 4) {
        storage_pose_data_received_ = false; // Data not valid
        return;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // Store the received pose data in class members
        x_mid_storage = msg->data[0];
        y_mid_storage = msg->data[1];
        z_storage = msg->data[2];
        alfa = msg->data[3]; // Assuming the 4th element is the yaw angle
        storage_pose_data_received_ = true; // Indicate valid storage data received
    } // End mutex scope

    // --- Visualization Part ---
    visualization_msgs::msg::Marker storage_marker;
    storage_marker.header.frame_id = "world";
    storage_marker.header.stamp = node_->now();
    storage_marker.ns = "storage_object";
    storage_marker.id = 0; // Unique ID for this marker namespace
    storage_marker.action = visualization_msgs::msg::Marker::ADD;
    storage_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE; // Use MESH_RESOURCE
    storage_marker.mesh_resource = "package://mycobot_gazebo/models/storage/collision/storage.stl"; // Path to the mesh

    // Set the pose using the received camera data
    {
         std::lock_guard<std::mutex> lock(data_mutex_); // Lock again to access stored data
         storage_marker.pose.position.x = x_mid_storage;
         storage_marker.pose.position.y = y_mid_storage;
         storage_marker.pose.position.z = z_storage; // Using the Z from camera data

         // Convert yaw (alfa) to quaternion
         tf2::Quaternion quaternion;
         quaternion.setRPY(0, 0, alfa); // Set roll=0, pitch=0, yaw=alfa
         storage_marker.pose.orientation.x = quaternion.x();
         storage_marker.pose.orientation.y = quaternion.y();
         storage_marker.pose.orientation.z = quaternion.z();
         storage_marker.pose.orientation.w = quaternion.w();
    }


    // Set scale (often 1,1,1 if the mesh has correct units)
    storage_marker.scale.x = 1.0;
    storage_marker.scale.y = 1.0;
    storage_marker.scale.z = 1.0;

    // Set color (e.g., a light grey)
    storage_marker.color.r = 0.8f;
    storage_marker.color.g = 0.8f;
    storage_marker.color.b = 0.8f;
    storage_marker.color.a = 0.7f; // Slightly transparent

    // Set lifetime so it doesn't disappear immediately
    storage_marker.lifetime = rclcpp::Duration(0, 0); // Stays for 2 seconds, then refreshed

    // Publish the marker (wrap in MarkerArray)
    visualization_msgs::msg::MarkerArray marker_array_storage;
    marker_array_storage.markers.push_back(storage_marker);
    marker_pub_->publish(marker_array_storage);

}

// Callback for button1 (Spawn Workpieces/Setup Scene)
void MTCTaskNode::spawn_workpieces_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        if (!workpieces_spawned_) {
            // Require storage pose data before setting up the scene
            if (!storage_pose_data_received_) {
                 RCLCPP_WARN(LOGGER, "Received button1 press, but storage pose data has not been received yet. Cannot setup planning scene.");
                 return;
            }
            RCLCPP_INFO(LOGGER, "Received button1 press (true). Attempting to setup planning scene...");
            setupPlanningScene();
            workpieces_spawned_ = true;
            task_planned_executed_ = false;
        } else {
            RCLCPP_INFO(LOGGER, "Received button1 press (true), but workpieces already spawned.");
            task_planned_executed_ = false;
        }
    }
}

// Callback for button2 (Run MTC)
void MTCTaskNode::run_mtc_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
     if (msg->data) {
        if (!workpieces_spawned_) {
            RCLCPP_WARN(LOGGER, "Received button2 press (true), but workpieces are not yet spawned.");
            return;
        }
         if (task_planned_executed_) {
             RCLCPP_INFO(LOGGER, "Received button2 press (true), but MTC task already completed this cycle.");
             return;
         }

        {
             std::lock_guard<std::mutex> lock(data_mutex_);
             if (!circles_data_received_) {
                 RCLCPP_WARN(LOGGER, "Received button2 press (true), but no circles data has been received yet. Cannot set target place pose or pick objects.");
                 return;
             }
        }

        RCLCPP_INFO(LOGGER, "Received button2 press (true). Running MTC task...");
        doTask();
     }
}

std::optional<geometry_msgs::msg::Pose> MTCTaskNode::get_start_storage_pose()
{
    const std::string target_frame = "world";
    const std::string source_frame = "start_storage";

    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
        transform_stamped = tf_buffer_->lookupTransform(
            target_frame, source_frame,
            tf2::TimePointZero, 5s);

        geometry_msgs::msg::Pose pose;

        pose.position.x = transform_stamped.transform.translation.x;
        pose.position.y = transform_stamped.transform.translation.y;
        pose.position.z = transform_stamped.transform.translation.z;

        pose.orientation = transform_stamped.transform.rotation;

        RCLCPP_INFO(LOGGER, "TF Lookup successful for %s to %s. Position: (%.3f, %.3f, %.3f), Orientation: (%.3f, %.3f, %.3f, %.3f)",
                    source_frame.c_str(), target_frame.c_str(),
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        return pose;

    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(LOGGER, "Could not transform %s to %s: %s",
                    source_frame.c_str(), target_frame.c_str(), ex.what());
        return std::nullopt;
    }
}


void MTCTaskNode::setupPlanningScene()
{
    RCLCPP_INFO(LOGGER, "Setting up planning scene...");
    moveit::planning_interface::PlanningSceneInterface psi;

    // // Clear any old objects
    // std::vector<std::string> object_ids_to_remove;
    // object_ids_to_remove.push_back("target_storage");
    // for(size_t i=0; i<6; ++i) object_ids_to_remove.push_back("object_" + std::to_string(i));
    // psi.removeCollisionObjects(object_ids_to_remove);
    // RCLCPP_INFO(LOGGER, "Removed previous collision objects.");


    // moveit_msgs::msg::CollisionObject storage_object;
    // storage_object.header.frame_id = "world";
    // storage_object.id = "target_storage";

    // shapes::Mesh* raw_mesh = shapes::createMeshFromResource("package://mycobot_gazebo/models/storage/collision/storage.stl");

    // if (!raw_mesh) {
    //     RCLCPP_ERROR(LOGGER, "Failed to load storage mesh from resource. Cannot add storage collision object.");
    //     target_storage_exists_ = false;
    //     // Continue to add smaller objects if TF is available, or return.
    //     // Let's add smaller objects if TF is available, even if storage mesh failed.
    // } else {
    //     shape_msgs::msg::Mesh converted_mesh;
    //     shapes::ShapeMsg mesh_msg;
    //     shapes::constructMsgFromShape(raw_mesh, mesh_msg);
    //     converted_mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    //     geometry_msgs::msg::Pose mesh_pose;
    //     {
    //         std::lock_guard<std::mutex> lock(data_mutex_);
    //         // Use the pose data received from the camera callback
    //         mesh_pose.position.x = x_mid_storage;
    //         mesh_pose.position.y = y_mid_storage;
    //         // Use the Z from camera data, or a fixed Z if the camera data Z is unreliable for the base
    //         mesh_pose.position.z = z_storage; // Using Z from camera data
    //          // If the camera detects the top surface Z, you might need to subtract half the storage height
    //          // For example, if storage height is 0.05m: mesh_pose.position.z = z_storage - 0.025;
    //          // The provided Z_storage=0.9001 suggests a fixed world Z, maybe ignore camera Z here?
    //          // Let's stick to using the camera Z as requested, but be aware of potential issues.
    //         // mesh_pose.position.z = 0.9001; // Alternative: use fixed Z

    //         // Set orientation based on the yaw angle from camera data
    //         tf2::Quaternion quaternion;
    //         quaternion.setRPY(0, 0, alfa); // Set roll=0, pitch=0, yaw=alfa
    //         mesh_pose.orientation.x = quaternion.x();
    //         mesh_pose.orientation.y = quaternion.y();
    //         mesh_pose.orientation.z = quaternion.z();
    //         mesh_pose.orientation.w = quaternion.w();

    //         RCLCPP_INFO(LOGGER, "Adding target_storage mesh at (%.3f, %.3f, %.3f) with yaw %.3f rad.",
    //                     mesh_pose.position.x, mesh_pose.position.y, mesh_pose.position.z, alfa);
    //     }


    //     storage_object.meshes.push_back(converted_mesh);
    //     storage_object.mesh_poses.push_back(mesh_pose);
    //     storage_object.operation = storage_object.ADD;

    //     std::vector<moveit_msgs::msg::CollisionObject> collision_objects = { storage_object };
    //     psi.addCollisionObjects(collision_objects);
    //     target_storage_exists_ = true;

    //     delete raw_mesh;
    // }


    if (start_storage_pose_) {
        const geometry_msgs::msg::Pose& storage_pose = start_storage_pose_.value();

        tf2::Quaternion q(
            storage_pose.orientation.x,
            storage_pose.orientation.y,
            storage_pose.orientation.z,
            storage_pose.orientation.w);

        double yaw = tf2::getYaw(q);

        std::vector<float> x_diff;
        std::vector<float> y_diff;
        float z_offset;

        const double PI = M_PI;
        bool is_yaw_0_or_180 = std::abs(yaw) < 0.1 || std::abs(yaw - PI) < 0.1 || std::abs(yaw + PI) < 0.1;
        bool is_yaw_plus_minus_90 = std::abs(yaw - PI/2.0) < 0.1 || std::abs(yaw + PI/2.0) < 0.1;


        if (is_yaw_0_or_180) {
            RCLCPP_INFO(LOGGER, "Detected TF Yaw close to 0 or 180 degrees (%.2f rad) for smaller objects layout.", yaw);
            x_diff = {-0.0325, -0.0325, -0.0325, 0.0325, 0.0325, 0.0325};
            y_diff = {-0.06, 0, 0.06, -0.06, 0, 0.06};
            z_offset = 0.925; // This Z offset seems related to the TF's height
        } else if (is_yaw_plus_minus_90) {
             RCLCPP_INFO(LOGGER, "Detected TF Yaw close to 90 or -90 degrees (%.2f rad) for smaller objects layout.", yaw);
             x_diff = {-0.06, 0, 0.06, -0.06, 0, 0.06};
             y_diff = {-0.0325, -0.0325, -0.0325, 0.0325, 0.0325, 0.0325};
             z_offset = 0.925; // This Z offset seems related to the TF's height
        }
         else {
             RCLCPP_WARN(LOGGER, "Detected TF Yaw is %.2f rad, which is not close to 0, 90, 180, or -90 degrees. Cannot determine smaller object layout based on TF.", yaw);
             // Do not add smaller objects if TF orientation is unexpected
             return;
        }

        if (x_diff.size() != 6 || y_diff.size() != 6) {
             RCLCPP_ERROR(LOGGER, "Internal error: Diff vector sizes are incorrect (%zu, %zu). Cannot add smaller collision objects.", x_diff.size(), y_diff.size());
             return;
        }

        std::vector<moveit_msgs::msg::CollisionObject> objects_to_add;
        objects_to_add.reserve(6); // Reserve space for efficiency

        for (size_t i = 0; i < 6; ++i) {
            moveit_msgs::msg::CollisionObject object;
            geometry_msgs::msg::Pose object_pose;

            object.id = "object_" + std::to_string(i);
            object.header.frame_id = "world";

            shape_msgs::msg::SolidPrimitive cylinder1, cylinder2;
            cylinder1.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            cylinder2.type = shape_msgs::msg::SolidPrimitive::CYLINDER;

            // Cylinder 1: Height = 0.033, Radius = 0.008
            cylinder1.dimensions.resize(2);
            cylinder1.dimensions[0] = 0.033;
            cylinder1.dimensions[1] = 0.008;

            // Cylinder 2: Height = 0.01, Radius = 0.021
            cylinder2.dimensions.resize(2);
            cylinder2.dimensions[0] = 0.01;
            cylinder2.dimensions[1] = 0.021;

            // Create poses
            geometry_msgs::msg::Pose cylinder1_pose;
            cylinder1_pose.orientation.w = 1.0;
            cylinder1_pose.position.z = (cylinder1.dimensions[0] / 2.0) + (cylinder2.dimensions[0] / 2.0); 

            geometry_msgs::msg::Pose cylinder2_pose;
            cylinder2_pose.orientation.w = 1.0;
            cylinder2_pose.position.z = 0;  // Move this cylinder downward

            // Add to collision object
            object.primitives.push_back(cylinder1);
            object.primitive_poses.push_back(cylinder1_pose);

            object.primitives.push_back(cylinder2);
            object.primitive_poses.push_back(cylinder2_pose);


            // Calculate position relative to the TF frame pose
            if (is_yaw_plus_minus_90) {
                object_pose.position.x = storage_pose.position.x + x_diff[i];
                object_pose.position.y = storage_pose.position.y + y_diff[i];
            } else { // Assuming is_yaw_0_or_180
                 object_pose.position.x = storage_pose.position.x - y_diff[i];
                 object_pose.position.y = storage_pose.position.y + x_diff[i];
            }

            object_pose.position.z = (cylinder2.dimensions[0] / 2.0) + z_offset; // Z relative to TF Z base + cylinder half height

            object_pose.orientation = storage_pose.orientation; // Inherit orientation from TF

            object.pose = object_pose;

            object.operation = moveit_msgs::msg::CollisionObject::ADD;

            objects_to_add.push_back(object);

            RCLCPP_INFO(LOGGER, "Adding object %zu at (%.3f, %.3f, %.3f) based on TF.",
                        i, object_pose.position.x, object_pose.position.y, object_pose.position.z);
        }

        if (!objects_to_add.empty()) {
             psi.applyCollisionObjects(objects_to_add);
             RCLCPP_INFO(LOGGER, "Applied %zu smaller collision objects based on TF.", objects_to_add.size());
        } else {
             RCLCPP_WARN(LOGGER, "No smaller collision objects were generated to apply based on TF.");
        }

    } else {
        RCLCPP_WARN(LOGGER, "Start storage TF pose not available. Cannot add smaller collision objects.");
    }
}

void MTCTaskNode::doTask()
{
  { 
      std::lock_guard<std::mutex> lock(data_mutex_); 

      if (!circles_data_received_) { 
          RCLCPP_ERROR(LOGGER, "No camera data received yet. Cannot set target place pose for MTC."); 
          return;
      }
      if (std::abs(data_array[5][0]) < 1e-6 && std::abs(data_array[5][1]) < 1e-6 && std::abs(data_array[5][2]) < 1e-6) { 
           RCLCPP_ERROR(LOGGER, "Camera data for object 5 appears invalid (all zeros). Cannot set target place pose for MTC.");
           return; 
      }

      target_place_pose_.header.frame_id = "world"; 
      target_place_pose_.header.stamp = node_->now(); 
      target_place_pose_.pose.position.x = data_array[5][0];
      target_place_pose_.pose.position.y = data_array[5][1];
      target_place_pose_.pose.position.z = data_array[5][2] + 0.005 + 0.001; // Adjusted height
      target_place_pose_.pose.orientation.w = 1.0; 

      RCLCPP_INFO(LOGGER, "Setting target place pose for THIS task run to (%.3f, %.3f, %.3f)",
                  target_place_pose_.pose.position.x, 
                  target_place_pose_.pose.position.y, 
                  target_place_pose_.pose.position.z);
  }

  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e.what()); 
    return; 
  }

  RCLCPP_INFO(LOGGER, "Task initialized. Planning..."); 

  if (!task_.plan(5)) 
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  RCLCPP_INFO(LOGGER, "Task planning succeeded. Publishing solution..."); 
  task_.introspection().publishSolution(*task_.solutions().front()); 

  RCLCPP_INFO(LOGGER, "Executing task..."); 
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) { 
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with code: " << result.val); 
    return; 
  }

  RCLCPP_INFO(LOGGER, "Task executed successfully."); 
  task_planned_executed_ = true; 
}

mtc::Task MTCTaskNode::createTask()
{
// ======================================================================================================== //
// ========================================= PICK AND PLACE =============================================== //
// ======================================================================================================== //
  mtc::Task task;
  task.stages()->setName("Pick and Place Task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm_controller";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "ee_link";
  const auto& eef = "eef_gripper";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", eef);
  task.setProperty("ik_frame", hand_frame);

  std::string object_name = "object_5";

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  #pragma GCC diagnostic pop
  mtc::Stage* current_state_ptr = nullptr; 

  // ================================ GET STARTING POSITION (CURRENT) ================================ //
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.1);
  cartesian_planner->setMaxAccelerationScalingFactor(0.1);
  cartesian_planner->setStepSize(.01);

  // ======================================== OPEN GRIPPER ============================================== //
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));


  // ======================================== MOVE TO PICK ============================================== //
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // Forward attach_object_stage to place pose generator
  mtc::Stage* attach_object_stage =
      nullptr;
  // ========================================= PICK OBJECT =============================================== //
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });
  // ======================================== APPROACH OBJECT ============================================== //
    {
      auto stage =
        std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
  // ========================================== GRASP OBJECT ================================================ //
  {
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose");
    stage->setPreGraspPose("open");
    stage->setObject(object_name);
    stage->setAngleDelta(M_PI / 12);
    stage->setMonitoredStage(current_state_ptr);

    // This is the transform from the object frame to the end-effector frame
    Eigen::Isometry3d grasp_frame_transform;
    // Initial orientation for the grasp relative to the object
    Eigen::Quaterniond q_initial = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    grasp_frame_transform.linear() = q_initial.matrix();
    // The grassping distance (front, y-right/left)
    grasp_frame_transform.translation().z() = 0.095;
    // The grassping height
    grasp_frame_transform.translation().x() = -0.025;


    // Compute IK
    auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setMinSolutionDistance(1.0);
    // The ComputeIK stage will now use the modified grasp_frame_transform
    wrapper->setIKFrame(grasp_frame_transform, hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    grasp->insert(std::move(wrapper));
  }
  // ======================================== ALLOW COLLISION ============================================== //
  // ================================= *to connect gripper with object ===================================== //
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand+object)");
      stage->allowCollisions(object_name,
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }
  // ========================================= CLOSE GRIPPER =============================================== //
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("closed");
      grasp->insert(std::move(stage));
    }
  // ========================================= ATTACHE OBJECT =============================================== //
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
  // =========================================== LIFT OBJECT ================================================ //
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object, support_surface)");
      stage->allowCollisions(object_name,
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }
  // ======================================== MOVE TO PLACE ============================================== //
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
  // ======================================== PLACE OBJECT ============================================== //
  {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
      place->properties().configureInitFrom(mtc::Stage::PARENT,
                                            { "eef", "group", "ik_frame" });

  // ======================================== LOWER OBJECT ============================================== //
      {
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(object_name);

        stage->setPose(target_place_pose_);
        stage->setMonitoredStage(attach_object_stage);  

        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(2);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(object_name);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        place->insert(std::move(wrapper));
      }
  // ======================================== OPEN GRIPPER ============================================== //
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        stage->setGroup(hand_group_name);
        stage->setGoal("open");
        place->insert(std::move(stage));
      }
  // ========================================= DETACH OBJECT ============================================= //
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        stage->detachObject(object_name, hand_frame);
        // attach_object_stage = stage.get();
        place->insert(std::move(stage));
      }
    task.add(std::move(place));
  }
  // =========================================== GO HOME ================================================ //
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("home");
    task.add(std::move(stage));
  }
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  RCLCPP_INFO(LOGGER, "MTC node is spinning and waiting for button presses...");

  spin_thread->join();

  rclcpp::shutdown();
  return 0;
}