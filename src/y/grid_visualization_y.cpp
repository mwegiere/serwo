#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <serwo/ErrorInfo.h>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <tf/transform_listener.h>

//Define a marker to be published in a topic and visualize in Rviz
//Used to visualize grid plate
class MarkerVis
{
public:
    MarkerVis(ros::NodeHandle &nh);
    //get info about object form topic
    void error_info_callback(const serwo::ErrorInfo& msg);
    //create a marker
    void setMarker();
    void vectorToTransform(std::vector<double>, int);
    void printTransform(tf::Transform);
    void printVector(tf::Vector3 V);
    void printMatrix3x3(tf::Matrix3x3);
    void listener();
   
private:
    double error_y;
    int found;
    //subscribe info. about object
    ros::Subscriber marker_info_sub;

    //publish info about marker
    ros::Publisher marker_vis_pub;
    ros::NodeHandle nh_;
    //info about marker
    visualization_msgs::Marker marker;

};

//set T_bG matrix
void MarkerVis::error_info_callback(const serwo::ErrorInfo& msg)
{
   error_y = msg.error;
   found = msg.found;
}
  
void MarkerVis::setMarker()
  {

    marker.header.frame_id = "/p_c_optical_frame";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    if(found == 1){
        marker.pose.position.x = error_y;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.5;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

    }
    else{
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    marker_vis_pub.publish( marker );
  }

MarkerVis::MarkerVis(ros::NodeHandle &nh)
  {
    nh_ = nh;
    error_y = 0.0;
    found = 0;
    marker_info_sub = nh_.subscribe("error", 1, &MarkerVis::error_info_callback, this);
    marker_vis_pub = nh_.advertise<visualization_msgs::Marker>( "object_seen_by_camera_in_rviz2", 1 );
  }

int main(int argc, char **argv) {  
  ros::init(argc, argv,"object_seen_by_camera_in_rviz");
  ros::NodeHandle nh;
  MarkerVis marker(nh);
  while (ros::ok())
  {
    marker.setMarker();
    ros::spinOnce();
  }

  return 0;
}

