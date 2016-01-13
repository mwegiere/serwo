#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <serwo/SerwoInfo.h>
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
    void chatterCallback(const serwo::SerwoInfo& msg);
    //create a marker
    void setMarker();
    void vectorToTransform(std::vector<double>, int);
    void printTransform(tf::Transform);
    void printVector(tf::Vector3 V);
    void printMatrix3x3(tf::Matrix3x3);
    void listener();
   
private:
    //position of object in /camera coordinate frame
    std::vector<double> vector_;
    //tell if an object was found by camera or not
    int found_;
    //subscribe info. about object
    ros::Subscriber marker_info_sub;
    //subscribe info. from TF about camera pos in /world
    tf::TransformListener tf_listener;
    //publish info about marker
    ros::Publisher marker_vis_pub;
    ros::NodeHandle nh_;
    //info about marker
    visualization_msgs::Marker marker;
    //quaternion - position of camera in /world frame
    tf::Quaternion rotation_bC;
    //translation - position of camera in /world frame
    tf::Vector3 translation_bC;

    tf::Transform T_gC;
    tf::Transform T_bC;
};

void MarkerVis::printTransform(tf::Transform  T)
{
    printf("[ %f %f %f %f \n %f %f %f %f \n %f %f %f %f \n %f %f %f %f]\n", T.getBasis()[0][0], T.getBasis()[0][1], T.getBasis()[0][2], T.getBasis()[0][3],
            T.getBasis()[1][0], T.getBasis()[1][1], T.getBasis()[1][2], T.getBasis()[1][3],
            T.getBasis()[2][0], T.getBasis()[2][1], T.getBasis()[2][2], T.getBasis()[2][3],
            T.getBasis()[3][0], T.getBasis()[3][1], T.getBasis()[3][2], T.getBasis()[3][3]);
}

void MarkerVis::printVector(tf::Vector3 V)
{
    printf("[ %f %f %f]\n", *(V), *(V+1), *(V+2));
}

void MarkerVis::printMatrix3x3(tf::Matrix3x3 M)
{
    printf("[ %f %f %f\n %f %f %f\n %f %f %f]\n ", M[0][0], M[0][1], M[0][2], M[1][0], M[1][1], M[1][2], M[2][0], M[2][1], M[2][2]);
}

//Change 16-elements vector to 4x4 matrix
void MarkerVis::vectorToTransform(cv::vector<double> v, int found)
  {
    if (found == 1)
    {
      tf::Vector3 translation_gC = tf::Vector3(v[3], v[7], v[11]);
      tf::Matrix3x3 rotation_gC = tf::Matrix3x3(v[0], v[1], v[2], v[4], v[5], v[6], v[8], v[9], v[10]);
      T_gC = tf::Transform(rotation_gC, translation_gC);
    }
    else
    {
      tf::Vector3 translation_gC = tf::Vector3(0, 0, 0);
      tf::Matrix3x3 rotation_gC = tf::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
      T_gC = tf::Transform(rotation_gC, translation_gC);
      std::cout<<"not found"<<std::endl;
    }
    return;
  }
  
//set T_bC matrix
void MarkerVis::listener()
{
  tf::StampedTransform transform;
    try{
      tf_listener.lookupTransform("/p_c_optical_frame", "/world",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    translation_bC = transform.getOrigin();
    rotation_bC = transform.getRotation();
    T_bC = tf::Transform(rotation_bC, translation_bC);
}

//set T_bG matrix
void MarkerVis::chatterCallback(const serwo::SerwoInfo& msg)
  {
    vector_ = msg.matrix;
    found_ = msg.found;
    vectorToTransform(vector_, found_);
  }
  
void MarkerVis::setMarker()
  {
    tf::Transform T_cB = T_bC.inverse();
    tf::Transform T_gB = T_cB * T_gC;
    tf::Quaternion q = T_gB.getRotation();
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = T_gB.getOrigin()[0];
    marker.pose.position.y = T_gB.getOrigin()[1];
    marker.pose.position.z = T_gB.getOrigin()[2];
    //std::cout<<T_gB.getOrigin()[2]<<std::endl;
    //printf("%f, %f, %f\n", T_gB.getOrigin()[0]*1000, T_gB.getOrigin()[1]*1000, T_gB.getOrigin()[2]*1000);
    marker.pose.orientation.x = q[0];
    marker.pose.orientation.y = q[1];
    marker.pose.orientation.z = q[2];
    marker.pose.orientation.w = q[3];
    printf("%f, %f, %f, %f\n", q[0], q[1], q[2], q[3]);
    marker.scale.x = 0.2;
    marker.scale.y = 0.4;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_vis_pub.publish( marker );
  }

MarkerVis::MarkerVis(ros::NodeHandle &nh)
  {
    nh_ = nh;
    marker_info_sub = nh_.subscribe("object_seen_by_camera", 1, &MarkerVis::chatterCallback, this);
    marker_vis_pub = nh_.advertise<visualization_msgs::Marker>( "object_seen_by_camera_in_rviz", 1 );
  }

int main(int argc, char **argv) {  
  ros::init(argc, argv,"object_seen_by_camera_in_rviz");
  ros::NodeHandle nh;
  MarkerVis marker(nh);
  while (ros::ok())
  {
    marker.setMarker();
    marker.listener();
    ros::spinOnce();
  }

  return 0;
}

