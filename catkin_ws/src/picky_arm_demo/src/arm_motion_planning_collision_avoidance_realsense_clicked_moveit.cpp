#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

// Global variables for storing camera parameters
sensor_msgs::CameraInfo camera_info;
bool camera_info_received = false;
bool octomap_clear = false;
bool trajectoryExecuted = false;
bool dropOff = false;
geometry_msgs::Point clickedPixel;
geometry_msgs::Point clickedPosition;
std_msgs::Bool triggerArm;

// Global variables for clicked point coordinates
int camera_width = 640;
int camera_length = 480;
float mm2m = 0.001;
float deg2rad = M_PI / 180.0;

ros::Publisher clicked_position_pub;
ros::Publisher trigger_arm_pub;

// Function to convert a geometry_msgs::Pose to a tf::Transform
tf::Transform get_tf(const geometry_msgs::Pose &pose)
{
    tf::Quaternion tf_quaternion;
    tf::quaternionMsgToTF(pose.orientation, tf_quaternion);

    tf::Transform tf_transform;
    tf_transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    tf_transform.setRotation(tf_quaternion);

    return tf_transform;
}

// Function to convert a tf::Transform to a geometry_msgs::Pose
geometry_msgs::Pose get_pose(const tf::Transform &tf_transform)
{
    geometry_msgs::Pose pose;

    tf::Vector3 tf_translation = tf_transform.getOrigin();
    pose.position.x = tf_translation.getX();
    pose.position.y = tf_translation.getY();
    pose.position.z = tf_translation.getZ();

    tf::Quaternion tf_quaternion = tf_transform.getRotation();
    tf::quaternionTFToMsg(tf_quaternion, pose.orientation);

    return pose;
}

// void executeTrajectoryCallback(const moveit_msgs::ExecuteTrajectoryFeedbackConstPtr &feedback)
// {
//     if (feedback->state == actionlib::SimpleClientGoalState::SUCCEEDED)
//     {
//         ROS_INFO("Trajectory completed ...");
//     }
//     // if (feedback->state == move_group::MoveGroupState::MONITOR)
//     // {
//     //     ROS_INFO("Trajectory is still executing...");
//     // }
//     // else if (feedback->state != move_group::MoveGroupState::IDLE)
//     // {
//     //     ROS_INFO("Trajectory Execution Completed.");
//     //     trajectoryExecuted = true;
//     //     // Now you can proceed with another task.
//     // }
// }

// Callback for the camera info message
void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    camera_info = *msg;
    camera_info_received = true;
}

// Callback for the depth image message
void depthImageCallback(const sensor_msgs::Image::ConstPtr &depth_image_msg)
{
    if (!camera_info_received)
    {
        ROS_WARN("Camera info not received yet. Skipping depth image callback.");
        return;
    }

    // Convert depth image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Check if the pixel coordinates are within the image bounds
    int pixel_x = static_cast<int>(clickedPixel.x * camera_width / 100.);  // scaling clickedoint is in range of 0-100
    int pixel_y = static_cast<int>(clickedPixel.y * camera_length / 100.); // scaling clickedoint is in range of 0-100
    if (pixel_x < 0 || pixel_x >= cv_ptr->image.cols || pixel_y < 0 || pixel_y >= cv_ptr->image.rows)
    {
        ROS_WARN("Pixel coordinates are out of bounds.");
        return;
    }

    // Get the depth value at the specified pixel coordinates
    float depth = cv_ptr->image.at<float>(pixel_y, pixel_x);

    // Check for invalid depth values
    if (depth <= 0.0)
    {
        // ROS_WARN("Waiting for a click or invalid depth value at the specified pixel coordinates.");
        return;
    }

    // Calculate the 3D point in camera coordinates
    float offset = 0.11;
    geometry_msgs::Pose pose_point_in_camera;
    pose_point_in_camera.position.x = mm2m * (pixel_x - camera_info.K[2]) * depth / camera_info.K[0];
    pose_point_in_camera.position.y = mm2m * (pixel_y - camera_info.K[5]) * depth / camera_info.K[4];
    pose_point_in_camera.position.z = (mm2m * depth) - offset;
    pose_point_in_camera.orientation.x = 0.0;
    pose_point_in_camera.orientation.y = 0.0;
    pose_point_in_camera.orientation.z = 0.0;
    pose_point_in_camera.orientation.w = 1.0;

    // Convert pose from camera from to robot frame.
    tf::Transform tf_point_in_camera = get_tf(pose_point_in_camera);

    // Initialize pose_robot_in_camera
    // Camera depth frame -> Base frame: [-0.3, 0.0, 0.5, 120.0, 0.0, 90.0]
    // geometry_msgs::Pose pose_robot_in_camera;
    // pose_robot_in_camera.position.x = -0.3;
    // pose_robot_in_camera.position.y = 0.0;
    // pose_robot_in_camera.position.z = 0.5;
    geometry_msgs::Pose pose_camera_in_robot;
    pose_camera_in_robot.position.x = -0.275;
    pose_camera_in_robot.position.y = 0.0;
    pose_camera_in_robot.position.z = 0.5;
    // Rotation matrix
    // 0  -1/2       sqrt(3)/2
    // -1  0         0
    // 0  -sqrt(3)/2 -1/2
    pose_camera_in_robot.orientation.x = -0.6123769;
    pose_camera_in_robot.orientation.y = 0.6123769;
    pose_camera_in_robot.orientation.z = -0.3535456;
    pose_camera_in_robot.orientation.w = 0.3535456;

    // tf::Quaternion orientation;
    // // orientation.setRPY(60.0 * deg2rad, 0.0 * deg2rad, +90.0 * deg2rad); // Euler angles (0, 120, 90)
    // orientation.setRPY(-90.0 * deg2rad, 60.0 * deg2rad, 90.0 * deg2rad); //
    // tf::quaternionTFToMsg(orientation, pose_camera_in_robot.orientation);
    tf::Transform tf_camera_in_robot = get_tf(pose_camera_in_robot);

    // Compute pose_point_in_robot
    // tf::Transform tf_camera_in_robot = tf_robot_in_camera.inverse();
    tf::Transform tf_point_in_robot = tf_camera_in_robot * tf_point_in_camera;
    geometry_msgs::Pose pose_point_in_robot = get_pose(tf_point_in_robot);

    float x_point_robot = pose_point_in_robot.position.x;
    float y_point_robot = pose_point_in_robot.position.y;
    float z_point_robot = pose_point_in_robot.position.z;

    // Publish the calculated 3D point
    clickedPosition.x = x_point_robot;
    clickedPosition.y = y_point_robot;
    clickedPosition.z = z_point_robot;
    clicked_position_pub.publish(clickedPosition);

    // Print the updated XYZ coordinates here
    // ROS_INFO("Pixel (%d, %d) at Camera Frame: (%f, %f, %f)m", pixel_x, pixel_y, pose_point_in_camera.position.x, pose_point_in_camera.position.y, pose_point_in_camera.position.z);
    // ROS_INFO("Point in Robot Frame: (%f, %f, %f)m", pose_point_in_robot.position.x, pose_point_in_robot.position.y, pose_point_in_robot.position.z);
}

// Callback for the clicked point message
void clickedPixelCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    // Print the updated clicked point coordinates here
    clickedPixel = *msg;
}

// Callback for the clicked position message
void clickedPositionCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    // Print the updated clicked point coordinates here
    clickedPosition = *msg;
}

// Callback for the trigger arm
void triggerArmCallback(const std_msgs::Bool::ConstPtr &msg)
{
    // Print the updated clicked point coordinates here
    triggerArm = *msg;
}

bool clear_octomap(ros::ServiceClient &clearOctomapClient, std_srvs::Empty &srv)
{
    if (clearOctomapClient.call(srv))
    {
        ROS_INFO("Octomap cleared successfully.");
        ros::Duration(1.0).sleep();
        ROS_INFO("Wait for 1 sec to update Octomap ...");
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call clear_octomap service");
        return false;
    }
}

void move_to_rest()
{
    // Initialize MoveIt! interfaces
    moveit::planning_interface::MoveGroupInterface arm_group("arm_group");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO("Arm Group Setup Completed ... Going Home!");

    arm_group.setPlanningTime(10.0); // Time to plan

    // Set a target pose
    geometry_msgs::Pose rest_pose;
    rest_pose.orientation.x = -0.0; // -0.8509035
    rest_pose.orientation.y = -0.0;
    rest_pose.orientation.z = -0.0;
    rest_pose.orientation.w = 1.0; // 0.525322
    rest_pose.position.x = -0.1923;
    rest_pose.position.y = -0.1224;
    rest_pose.position.z = 0.1947;
    arm_group.setPoseTarget(rest_pose);

    ROS_INFO("MoveIt! is planning to rest pose ...");

    // Planing trajectory.
    ROS_INFO("Planning started ..!");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("*** Planning finished ***");

    if (success)
    {
        ROS_INFO("Planning succeeded. Now robot executes the trajectory!");
        // Execute trajectory.
        arm_group.execute(my_plan);
    }
    else
    {
        ROS_WARN("Planning Failed! Check if the target pose is reachable and that there are no collisions.");
    }
}

void move_to_dropoff()
{
    // Initialize MoveIt! interfaces
    moveit::planning_interface::MoveGroupInterface arm_group("arm_group");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO("Arm Group Setup Completed ... Going DropOff!");

    arm_group.setPlanningTime(10.0); // Time to plan

    // Set a target pose
    geometry_msgs::Pose dropoff_pose;
    dropoff_pose.orientation.x = -0.4754915;
    dropoff_pose.orientation.y = 0.8790473;
    dropoff_pose.orientation.z = -0.0026067;
    dropoff_pose.orientation.w = 0.0343051;
    dropoff_pose.position.x = -0.1865;
    dropoff_pose.position.y = -0.0235;
    dropoff_pose.position.z = +0.1981;
    arm_group.setPoseTarget(dropoff_pose);

    ROS_INFO("MoveIt! is planning to dropoff pose ...");

    // Planing trajectory.
    ROS_INFO("Planning started ..!");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("*** Planning finished ***");

    if (success)
    {
        ROS_INFO("Planning succeeded. Now robot executes the trajectory!");
        // Execute trajectory.
        arm_group.execute(my_plan);
    }
    else
    {
        ROS_WARN("Planning Failed! Check if the target pose is reachable and that there are no collisions.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Arm_Realsense");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Subscribe to the camera info topic
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/depth/camera_info", 1, cameraInfoCallback);

    // Subscribe to the depth image topic
    ros::Subscriber depth_image_sub = nh.subscribe("/camera/depth/image_rect_raw", 1, depthImageCallback);

    // Subscribe to the clicked point topic
    ros::Subscriber clicked_point_sub = nh.subscribe("/clicked_point", 1, clickedPixelCallback);

    // Create a publisher for the clicked position
    clicked_position_pub = nh.advertise<geometry_msgs::Point>("/clicked_position", 1);

    // Subscribe to the clicked point topic
    ros::Subscriber clicked_position_sub = nh.subscribe("/clicked_position", 1, clickedPositionCallback);

    // Subscribe to the trigger arm
    ros::Subscriber trigger_arm_sub = nh.subscribe("/trigger_arm", 1, triggerArmCallback);

    // Create a publisher for arm trigger
    trigger_arm_pub = nh.advertise<std_msgs::Bool>("/trigger_arm", 1);

    // Create a subscriber to monitor trajectory execution feedback
    // ros::Subscriber feedback_sub = nh.subscribe("/execute_trajectory/feedback", 10, executeTrajectoryCallback);

    // --- Move Arm to Position ---
    // Set up MoveIt! interfaces
    moveit::planning_interface::MoveGroupInterface arm_group("arm_group");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO("Arm Group Setup Completed.");

    // Clear octomap
    ros::service::waitForService("/clear_octomap");
    ros::ServiceClient clearOctomapClient = nh.serviceClient<std_srvs::Empty>("/clear_octomap");
    std_srvs::Empty srv;
    ROS_INFO("Ocotomap Service Client is Running.");

    move_to_rest();

    while (ros::ok())
    {
        octomap_clear = false;
        trajectoryExecuted = false;
        dropOff = false;

        if (triggerArm.data)
        {
            ROS_INFO("Arm triggered to move to clicked position.");
            // Free up arm to accept new trigger.
            std_msgs::Bool trigger_arm_msg;
            trigger_arm_msg.data = false;
            trigger_arm_pub.publish(trigger_arm_msg);

            // Clear octomap
            ros::service::waitForService("/clear_octomap");
            std_srvs::Empty srv;
            octomap_clear = clear_octomap(clearOctomapClient, srv);
        }
        if (octomap_clear)
        {
            // Specify the reference frame
            arm_group.setPlanningTime(10.0); // Time to plan

            // Set a target pose
            geometry_msgs::Pose target_pose;
            target_pose.orientation.x = +0.5;
            target_pose.orientation.y = -0.5;
            target_pose.orientation.z = +0.5;
            target_pose.orientation.w = -0.5;
            target_pose.position.x = clickedPosition.x;
            target_pose.position.y = clickedPosition.y;
            target_pose.position.z = clickedPosition.z;
            arm_group.setPoseTarget(target_pose);

            ROS_INFO("MoveIt! is planning to (%f, %f, %f)m in Robot Frame", clickedPosition.x, clickedPosition.y, clickedPosition.z);

            // Planing trajectory.
            ROS_INFO("Planning started ..!");
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
            ROS_INFO("*** Planning finished ***");

            if (success)
            {
                ROS_INFO("Planning succeeded. Now robot executes the trajectory!");
                // Execute trajectory.
                arm_group.execute(my_plan);
                trajectoryExecuted = true;
                while (!trajectoryExecuted)
                {
                    ros::Duration(0.1).sleep(); // Sleep for a short duration to avoid busy-waiting
                    ROS_INFO("Trajectory is being executed ...");
                }
                if (trajectoryExecuted)
                {
                    dropOff = true;
                    ROS_INFO("*** Trajectory Execution Completed ***");
                }
            }
            else
            {
                ROS_WARN("Planning Failed! Check if the target pose is reachable and that there are no collisions.");
            }

            if (dropOff)
            {
                move_to_dropoff();
            }

            if (dropOff && success)
            {
                // Move back robot to rest pose
                move_to_rest();
            }
        }
    }
    return 0;
}