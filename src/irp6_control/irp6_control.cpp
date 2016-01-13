#include <ros/ros.h>
//#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
//#include <serwo/SerwoInfo.h>
//#include <cmath>
//#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <tf/transform_listener.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>

#include <force_control_msgs/ForceControl.h>
#include <force_control_msgs/ToolGravityParam.h>

#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Vector3.h>

class Irp6Control {
 public:
  Irp6Control(ros::NodeHandle &nh);
  void run();
  ros::NodeHandle nh_;
  ros::Publisher fcl_param_publisher;
  ros::Publisher tg_param_publisher;
  ros::Rate loop_rate;
  force_control_msgs::ForceControl forceControlGoal;
  force_control_msgs::ToolGravityParam tg_goal;
};

Irp6Control::Irp6Control(ros::NodeHandle &nh) : loop_rate(500) {
  nh_ = nh;
  fcl_param_publisher = nh_.advertise<force_control_msgs::ForceControl>("/irp6p_arm/fcl_param", 1);
  tg_param_publisher = nh_.advertise<force_control_msgs::ToolGravityParam>("/irp6p_arm/tg_param", 1);
}

void Irp6Control::run() {

   forceControlGoal = force_control_msgs::ForceControl();

   //forceControlGoal.inertia = geometry_msgs::Inertia(inertia1, inertia2);
   geometry_msgs::Vector3 inertia1;
   inertia1.x = 0.0;
   inertia1.y = 0.0;
   inertia1.z = 0.0;

   geometry_msgs::Vector3 inertia2;
   inertia2.x = 0.0;
   inertia2.y = 0.0;
   inertia2.z = 0.0;

   force_control_msgs::Inertia inertia;
   inertia.translation = inertia1;
   inertia.rotation = inertia2;
   forceControlGoal.inertia = inertia;

   //forceControlGoal.reciprocaldamping = force_control_msgs::ReciprocalDamping(geometry_msgs::Vector3(0.0025, 0.0025, 0.0025), geometry_msgs::Vector3(0.0, 0.0, 0.0));
   geometry_msgs::Vector3 reciprocaldamping1;
   reciprocaldamping1.x = 0.0025;
   reciprocaldamping1.y = 0.0025;
   reciprocaldamping1.z = 0.0025;

   geometry_msgs::Vector3 reciprocaldamping2;
   reciprocaldamping2.x = 0.0;
   reciprocaldamping2.y = 0.0;
   reciprocaldamping2.z = 0.0;

   force_control_msgs::ReciprocalDamping reciprocaldamping;
   reciprocaldamping.translation = reciprocaldamping1;
   reciprocaldamping.rotation = reciprocaldamping2;
   forceControlGoal.reciprocaldamping = reciprocaldamping;

   //forceControlGoal.wrench = geometry_msgs::Wrench(geometry_msgs::Vector3(0.0, 0.0, 0.0), geometry_msgs::Vector3(0.0, 0.0, 0.0);
   geometry_msgs::Vector3 wrench1;
   wrench1.x = 0.0;
   wrench1.y = 0.0;
   wrench1.z = 0.0;

   geometry_msgs::Vector3 wrench2;
   wrench2.x = 0.0;
   wrench2.y = 0.0;
   wrench2.z = 0.0;

   geometry_msgs::Wrench wrench;
   wrench.force = wrench1;
   wrench.torque = wrench2;
   forceControlGoal.wrench = wrench;

   //forceControlGoal.twist = geometry_msgs::Twist(geometry_msgs::Vector3(0.0, 0.0, 0.0), geometry_msgs::Vector3(0.0, 0.01, 0.0));
   geometry_msgs::Vector3 twist1;
   twist1.x = 0.0;
   twist1.y = 0.0;
   twist1.z = 0.0;

   geometry_msgs::Vector3 twist2;
   twist2.x = 0.0;
   twist2.y = 0.01;
   twist2.z = 0.0;

   geometry_msgs::Twist twist;
   twist.linear = twist1;
   twist.angular = twist2;
   forceControlGoal.twist = twist;

   fcl_param_publisher.publish(forceControlGoal);
 
   
   tg_goal = force_control_msgs::ToolGravityParam();

   tg_goal.weight = 10.8;

   geometry_msgs::Vector3 mass_center;
   mass_center.x = 0.004;
   mass_center.y = 0.0;
   mass_center.z = 0.156;
   tg_goal.mass_center = mass_center;

   tg_param_publisher.publish(tg_goal);


   //self.conmanSwitch([self.robot_name+'mForceTransformation'], [], True)
   //time.sleep(0.05)
   //self.conmanSwitch([self.robot_name+'mForceControlLaw'], [], True)

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "serwovision_control");
  ros::NodeHandle nh;
  Irp6Control control(nh);

  while (ros::ok())
   {
   control.run();
   ros::spinOnce();
   //loop_rate.sleep();
   }
  return 0;
}

