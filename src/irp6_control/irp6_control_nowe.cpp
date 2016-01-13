#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <serwo/ErrorInfo.h>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <tf/transform_listener.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>

class Irp6Control {
 public:
  Irp6Control(ros::NodeHandle &nh);
  void run();
  ros::NodeHandle nh_;

private:
  double error_y; //actual error in y axis
  int found; //information whether object was found by camera (1), or not (0)
  int frequency;  //velocity steering frequency

  double minError_y;  //minimal error to calculate new velocity

  double newVel_y;  //new velocity in y axis
  double newAcceleration_y;  //new acceleration in y axis

  double p;  //p value for PID regulator
  double i;  //i value for PID regulator
  double d;  //d value for PID regulator

  double maxVel_y; //max velocity in y axis
  double maxAcceleration_y; //max acceleration in y axis
  double min_max_pos_cartesian_y; //position constraint in y axis (symetrical)

  double current_pos_cartesian_y; //actual position in y axis
  double new_pos_cartesian_y; //new position in y axis

  ros::Subscriber error_info_subscriber;
  ros::Subscriber pos_cartesian_y_subscriber;

  void callback();
  void calculateNewVel();
  void calculateNewCartesianPose();
  void calculateNewAcceleration();
  void checkVelocityLimits();
  void checkAccelerationLimits();
  void checkCartesianLimits();
  void setZeroValues();
  void setNewVelocity();
  void calculateAndSetNewValues();
  void TestLimits();
  void error_info_callback(const serwo::ErrorInfo&);
  void pos_cartesian_y_callback(const geometry_msgs::Pose& msg);
};

Irp6Control::Irp6Control(ros::NodeHandle &nh) {
  nh_ = nh;

  error_y = 0.0;
  found = 0;
  frequency = 500;

  minError_y = 0.01;

  newVel_y = 0.0;
  newAcceleration_y = 0.0;

  p = 0.1;
  i = 0.0;
  d = 0.0;

  maxVel_y = 0.05;
  maxAcceleration_y = 25.0;
  min_max_pos_cartesian_y = 0.3;

  current_pos_cartesian_y = 5.0;
  new_pos_cartesian_y = 0.0;

  error_info_subscriber = nh_.subscribe("error", 1,
                                        &Irp6Control::error_info_callback,
                                        this);

  pos_cartesian_y_subscriber = nh_.subscribe("/irp6p_arm/cartesian_position", 1,
                                        &Irp6Control::pos_cartesian_y_callback,
                                        this);
}

void Irp6Control::error_info_callback(const serwo::ErrorInfo& msg) {
  error_y = msg.error;
  found = msg.found;
}

void Irp6Control::pos_cartesian_y_callback(const geometry_msgs::Pose& msg) {
  current_pos_cartesian_y = msg.position.y;
}

void Irp6Control::calculateNewVel() {
    if (error_y > minError_y || error_y < -minError_y)
    {
        newVel_y = error_y * p;
    }
    else
    {
      newVel_y = 0.0;
    }
  }

void Irp6Control::calculateNewCartesianPose() {
  new_pos_cartesian_y = current_pos_cartesian_y + newVel_y * (1 / frequency);
}

void Irp6Control::calculateNewAcceleration() {
    newAcceleration_y = newVel_y * frequency;
}

void Irp6Control::checkVelocityLimits() {
    if (newVel_y > maxVel_y){
        newVel_y = maxVel_y;
    }

    if (newVel_y < -maxVel_y){
        newVel_y = -maxVel_y;
    }
}

void Irp6Control::checkAccelerationLimits() {
    if (newAcceleration_y > maxAcceleration_y){
        newVel_y = maxVel_y;
    }
    if (newAcceleration_y < -maxAcceleration_y){
        newVel_y = -maxVel_y;
    }
}

void Irp6Control::checkCartesianLimits() {
    if (new_pos_cartesian_y > min_max_pos_cartesian_y and new_pos_cartesian_y < -min_max_pos_cartesian_y) {
      newVel_y = 0.0;
    }
}

void Irp6Control::setZeroValues() {
    newVel_y = 0.0;
}

void Irp6Control::setNewVelocity() {
  checkVelocityLimits();
  checkCartesianLimits();
}

void Irp6Control::calculateAndSetNewValues() {
  if (found == 1) {
    calculateNewVel();
    calculateNewCartesianPose();
    setNewVelocity();
  } else
    setZeroValues();
}

void Irp6Control::run() {
    calculateAndSetNewValues();

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "serwovision_control");
  ros::NodeHandle nh;
  Irp6Control control(nh);

  //ros::spinOnce();

  while (ros::ok())
   {
   control.run();
   ros::spinOnce();
   }

  return 0;
}

