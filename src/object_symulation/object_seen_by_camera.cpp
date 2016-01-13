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
    void chatterCallback(const sensor_msgs::JointState& msg);
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
    double y_;
    //subscribe info. about object
    ros::Publisher grid_info_pub;
    //subscribe info. from TF about camera pos in /world
    tf::TransformListener tf_listener;
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
      printVector(translation_gC);
      printMatrix3x3(rotation_gC);
      T_gC = tf::Transform(rotation_gC, translation_gC);
    }
    else
    {
      tf::Vector3 translation_gC = tf::Vector3(0, 0, 0);
      tf::Matrix3x3 rotation_gC = tf::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
      T_gC = tf::Transform(rotation_gC, translation_gC);
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
void MarkerVis::chatterCallback(const sensor_msgs::JointState& msg)
  {
    y_ = msg.position;
    vector_ = std::vector<double>[1,0,0,0.88, 0,1,0,y_, 0,0,1,0.6, 0,0,0,1];
  }

void MarkerVis::setMarker()
  {

    vectorToTransform(vector_, found_);
    tf::Transform T_cB = T_bC.inverse();
    tf::Transform T_gB = T_cB * T_gC;
    printTransform(T_gB);
    tf::Quaternion q = T_gB.getRotation();

    marker_vis_pub.publish( marker );
  }

MarkerVis::MarkerVis(ros::NodeHandle &nh)
  {
    nh_ = nh;
    marker_info_sub = nh_.subscribe("/conveyor/joint_states", 1, &MarkerVis::chatterCallback, this);
    grid_info_pub = nh_.advertise<serwo::SerwoInfo>( "object_seen_by_camera", 1 );
    found_ = 1;
  }

int main(int argc, char **argv) {  
  ros::init(argc, argv,"object_seen_by_camera");
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

