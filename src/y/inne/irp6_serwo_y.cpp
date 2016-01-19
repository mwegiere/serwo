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

#include <force_control_msgs/ForceControl.h>
#include <force_control_msgs/ToolGravityParam.h>

#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Vector3.h>

#include <fstream>

class Irp6Control {
 public:
  Irp6Control(ros::NodeHandle &nh);
  void run();
  ros::NodeHandle nh_;
  ros::Rate loop_rate;


  double error_y; //actual error in y axis

  double e2, e1, e0, u2, u1, u0;  // variables used in PID computation
  double r;  // command
  double y;  // plant output
  // -- these parameters should be user-adjustable
  double Kp = 0.012622295959545;   // proportional gain
  double Ki = 0.0000526235234073302;	// integral gain
  double Kd = 0.672688766296438;  	// derivative gain
  int N = 75.5037031133956;   		// filter coefficients
  double Ts = 0.002;   // This must match actual sampling time PID

  double a0, a1, a2, b0, b1, b2, ku0, ku1, ku2, kw0, ke0, ke1, ke2;

 private:

  double pre_error_y;
  int found; //information whether object was found by camera (1), or not (0)
  int frequency;  //velocity steering frequency
  double dt;

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

  ros::Publisher fcl_param_publisher;
  ros::Publisher tg_param_publisher;
  
  force_control_msgs::ForceControl forceControlGoal;
  force_control_msgs::ToolGravityParam tg_goal;

  geometry_msgs::Vector3 inertia1;
  geometry_msgs::Vector3 inertia2;
  force_control_msgs::Inertia inertia;
  geometry_msgs::Vector3 reciprocaldamping1;
  geometry_msgs::Vector3 reciprocaldamping2;
  force_control_msgs::ReciprocalDamping reciprocaldamping;
  geometry_msgs::Vector3 wrench1;
  geometry_msgs::Vector3 wrench2;
  geometry_msgs::Wrench wrench;

  geometry_msgs::Vector3 twist1;
  geometry_msgs::Vector3 twist2;
  geometry_msgs::Twist twist;

  geometry_msgs::Vector3 mass_center;

  void VelSteering(double);
  void callback();
  void calculateNewVel();
  void calculateNewCartesianPose();
  void calculateNewAcceleration();
  void checkVelocityLimits();
  void checkCartesianLimits();
  void setZeroValues();
  void setNewVelocity();
  void calculateAndSetNewValues();
  void TestLimits();
  void error_info_callback(const serwo::ErrorInfo&);
  void pos_cartesian_y_callback(const geometry_msgs::Pose& msg);
};

Irp6Control::Irp6Control(ros::NodeHandle &nh) : loop_rate(500){
  nh_ = nh;

  error_y = 0.0;
  pre_error_y = 0.0;
  found = 0;
  frequency = 500;
  dt = 0.002;

  minError_y = 0.01;

  newVel_y = 0.0;
  newAcceleration_y = 0.0;

  p = 0.012622295959545;
  i = 0.000052623523407;
  d = 0.672688766296438;

  maxVel_y = 0.05;
  min_max_pos_cartesian_y = 0.3;

  current_pos_cartesian_y = 5.0;
  new_pos_cartesian_y = 0.0;

  error_info_subscriber = nh_.subscribe("error", 1,
                                        &Irp6Control::error_info_callback,
                                        this);

  pos_cartesian_y_subscriber = nh_.subscribe("/irp6p_arm/cartesian_position", 1,
                                        &Irp6Control::pos_cartesian_y_callback,
                                        this);

  fcl_param_publisher = nh_.advertise<force_control_msgs::ForceControl>("/irp6p_arm/fcl_param", 1);
  tg_param_publisher = nh_.advertise<force_control_msgs::ToolGravityParam>("/irp6p_arm/tg_param", 1);

  forceControlGoal = force_control_msgs::ForceControl();
  tg_goal = force_control_msgs::ToolGravityParam();

  a0 = (1+N*Ts);
  a1 = -(2 + N*Ts);
  a2 = 1;
  b0 = Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N;
  b1 = -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N);
  b2 = Kp + Kd*N;
  ku1 = a1/a0; ku2 = a2/a0; ke0 = b0/a0; ke1 = b1/a0; ke2 = b2/a0;


}

void Irp6Control::VelSteering(double v_y){

    inertia1.x = 0.0;
    inertia1.y = 0.0;
    inertia1.z = 0.0;
    inertia2.x = 0.0;
    inertia2.y = 0.0;
    inertia2.z = 0.0;

    inertia.translation = inertia1;
    inertia.rotation = inertia2;
    forceControlGoal.inertia = inertia;

    reciprocaldamping1.x = 0.0;
    reciprocaldamping1.y = 0.0;
    reciprocaldamping1.z = 0.0;

    reciprocaldamping2.x = 0.0;
    reciprocaldamping2.y = 0.0;
    reciprocaldamping2.z = 0.0;

    reciprocaldamping.translation = reciprocaldamping1;
    reciprocaldamping.rotation = reciprocaldamping2;
    forceControlGoal.reciprocaldamping = reciprocaldamping;

    wrench1.x = 0.0;
    wrench1.y = 0.0;
    wrench1.z = 0.0;
    wrench2.x = 0.0;
    wrench2.y = 0.0;
    wrench2.z = 0.0;

    wrench.force = wrench1;
    wrench.torque = wrench2;
    forceControlGoal.wrench = wrench;

    twist1.x = 0.0;
    twist1.y = v_y;
    twist1.z = 0.0;
    twist2.x = 0.0;
    twist2.y = 0.0;
    twist2.z = 0.0;

    twist.linear = twist1;
    twist.angular = twist2;
    forceControlGoal.twist = twist;

    fcl_param_publisher.publish(forceControlGoal);

    tg_goal.weight = 10.8;

    mass_center.x = 0.004;
    mass_center.y = 0.0;
    mass_center.z = 0.156;
    tg_goal.mass_center = mass_center;

    tg_param_publisher.publish(tg_goal);
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
        newVel_y = error_y * 5;
    }
    else
    {
      newVel_y = 0.0;
    }

    /*if (error_y > minError_y || error_y < -minError_y){
        e2=e1; e1=e0; u2=u1; u1=u0; // update variables
        e0 = error_y;  // compute new error
        u0 = -ku1*u1 - ku2*u2 + ke0*e0 + ke1*e1 + ke2*e2;
        newVel_y = u0;
    }
    else{
        newVel_y = 0.0;
    }*/

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
    calculateNewAcceleration();
    setNewVelocity();
  } else
    setZeroValues();
}

void Irp6Control::run() {
    calculateAndSetNewValues();
    //std::cout<<newVel_y<<std::endl;
    VelSteering(newVel_y);
}

int main(int argc, char **argv) {
  std::ofstream error_time;
  error_time.open ("error.txt");

  ros::init(argc, argv, "serwovision_control2");
  ros::NodeHandle nh;
  Irp6Control control(nh);

  while (ros::ok())
   {
   error_time << control.error_y <<"\n";
   control.run();
   ros::spinOnce();
   control.loop_rate.sleep();
   }
  error_time.close();

  return 0;
}

