#include "OdometryHandler.h"


OdometryHandler::OdometryHandler(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ 

    initializeSubscribers(); 
    initializePublishers();
    
    //initialize variables here, as needed
    
    
}

void OdometryHandler::initializeSubscribers()
{
    cartographer_pose_sub_ = nh_.subscribe("/tracked_pose", 1, &OdometryHandler::cartographerPoseCallback,this);  
    camera_odom_sub_ = nh_.subscribe("/camera/odom/sample", 1, &OdometryHandler::cameraOdometryCallback,this); 
    mocap_odom_sub_ = nh_.subscribe("/vicon/dronelab/odom", 1, &OdometryHandler::mocapOdometryCallback,this);
}

void OdometryHandler::initializePublishers()
{
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 1, true); 
}


void OdometryHandler::cartographerPoseCallback(const geometry_msgs::PoseStamped& message_holder) {

    cartographer_pose_val_ = message_holder; // copy the received data into member variable, so ALL member funcs of ExampleRosClass can access it
}
void OdometryHandler::cameraOdometryCallback(const nav_msgs::Odometry& message_holder) {

    camera_odom_val_ = message_holder; 
}
void OdometryHandler::mocapOdometryCallback(const nav_msgs::Odometry& message_holder) {

    mocap_odom_val_ = message_holder; 
}

void OdometryHandler::OdometryWork()
{   
    nav_msgs::Odometry msg;
    msg.header = cartographer_pose_val_.header;
    msg.child_frame_id = camera_odom_val_.child_frame_id;
    msg.pose.pose.position.x = cartographer_pose_val_.pose.position.x;
    msg.pose.pose.position.y = cartographer_pose_val_.pose.position.y;
    msg.pose.pose.position.z = camera_odom_val_.pose.pose.position.z;
    msg.pose.pose.orientation = cartographer_pose_val_.pose.orientation;
    msg.twist = camera_odom_val_.twist;
    odometry_pub_.publish(msg);
    //ROS_INFO("Working");
}





int main(int argc, char** argv) 
{ 
    
    // ROS set-ups:
    ros::init(argc, argv, "odometry_handler_node"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor  
    OdometryHandler odometryHandler(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
    ros::Rate loop_rate(200);

    

    while (ros::ok())
    {
    ros::spinOnce();
    odometryHandler.OdometryWork();
    loop_rate.sleep();
    }

    return 0;
} 
