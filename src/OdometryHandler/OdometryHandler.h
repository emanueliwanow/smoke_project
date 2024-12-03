
#ifndef ODOMETRY_HANDLER
#define ODOMETRY_HANDLER

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> 
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


// define a class, including a constructor, member variables and member functions
class OdometryHandler
{
public:
    OdometryHandler(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor

    void OdometryWork();
    
private:
    
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    ros::Subscriber cartographer_pose_sub_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber mocap_odom_sub_;
    ros::Subscriber camera_odom_sub_;

    ros::Publisher  odometry_pub_;
    
    geometry_msgs::PoseStamped cartographer_pose_val_; 
    nav_msgs::Odometry mocap_odom_val_;
    nav_msgs::Odometry camera_odom_val_;
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    
    void cartographerPoseCallback(const geometry_msgs::PoseStamped& message_holder); //prototype for callback of example subscriber
    void cameraOdometryCallback(const nav_msgs::Odometry& message_holder);
    void mocapOdometryCallback(const nav_msgs::Odometry& message_holder);
}; 

#endif 