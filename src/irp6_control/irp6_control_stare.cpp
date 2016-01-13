#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <serwo/SerwoInfo.h>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <tf/transform_listener.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>

//Define a marker to be published in a topic and visualize in Rviz
//Used to visualize grid plate
//konwencja
//T_aB - pozycja ukladu a w ukladzie B
//w mojej notacji a stoi na dole, a B stoi na gorze
class Irp6Control {
 public:
  Irp6Control(ros::NodeHandle &nh);
  void run();
  ros::NodeHandle nh_;

private:
  //dane o obrazie otrzymane z topica
  std::vector<double> T_gC_vector;
  int found;

  std::vector<std::vector<double> > T_gC;

  int frequency;  //czestotliwosc wysylania sterowania dla robota

  std::vector<double> error;  //chyby dla 6 stopni swobody
  std::vector<double> minError;  //minimalny uchyb przy ktorym zostanie wyliczona nowa predkosc

  std::vector<double> newVel;  //wektro predkosci dla 6 stopni swobody
  std::vector<double> newAcceleration;  //wektro przyspieszenia dla 6 stopni swobody

  double p;  //wartosc p dla regulatora PID
  double i;  //wartosc p dla regulatora PID
  double d;  //wartosc p dla regulatora PID

  double maxVel;
  double maxAcceleration;
  std::vector<double> new_pos_cartesian;

  //kartezajanskie ograniczenia pozycji jako trojki xyz
  //w ukladzie zwiazanym z baza robota
  //przetestowac ograniczenia na prawdziwym robocie
  std::vector<double> min_pos;  //xyz
  std::vector<double> max_pos;  //xyz

  std::vector<double> w_vector[3][8];

  void moveToJointPosition(std::vector<double>, double);

  ros::Publisher error_pub;
  ros::Subscriber image_info_subscriber;

  void limits_test();
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
  void regulatroDecomposition();
  void vectorToTransform(std::vector<double>, int);
  void image_info_callback(const serwo::SerwoInfo&);
};

Irp6Control::Irp6Control(ros::NodeHandle &nh) {

  nh_ = nh;
  for (int ii = 0; ii < 4; ii++) {
    std::vector<double> row;
    for (int jj = 0; jj < 4; jj++) {
      row.push_back(1);
    }
    T_gC.push_back(row);
  }

  found = 1;

  frequency = 500;

  error.push_back(0);
  error.push_back(0);
  error.push_back(0);
  error.push_back(0);
  error.push_back(0);
  error.push_back(0);

  minError.push_back(0.01);
  minError.push_back(0.01);
  minError.push_back(0.01);
  minError.push_back(0.01);
  minError.push_back(0.01);
  minError.push_back(0.01);

  newVel.push_back(0);
  newVel.push_back(0);
  newVel.push_back(0);
  newVel.push_back(0);
  newVel.push_back(0);
  newVel.push_back(0);

  newAcceleration.push_back(0);
  newAcceleration.push_back(0);
  newAcceleration.push_back(0);
  newAcceleration.push_back(0);
  newAcceleration.push_back(0);
  newAcceleration.push_back(0);

  p.push_back(0);
  p.push_back(0.5);
  p.push_back(0);
  p.push_back(0);
  p.push_back(0);
  p.push_back(0);

  i.push_back(0);
  i.push_back(0);
  i.push_back(0);
  i.push_back(0);
  i.push_back(0);
  i.push_back(0);

  d.push_back(0);
  d.push_back(0);
  d.push_back(0);
  d.push_back(0);
  d.push_back(0);
  d.push_back(0);

  maxVel.push_back(0.05);
  maxVel.push_back(0.05);
  maxVel.push_back(0.05);
  maxVel.push_back(0.0);
  maxVel.push_back(0.0);
  maxVel.push_back(0.0);

  maxAcceleration.push_back(25.0);
  maxAcceleration.push_back(25.0);
  maxAcceleration.push_back(25.0);
  maxAcceleration.push_back(0.0);
  maxAcceleration.push_back(0.0);
  maxAcceleration.push_back(0.0);

  new_pos_cartesian.push_back(0.0);
  new_pos_cartesian.push_back(0.0);
  new_pos_cartesian.push_back(0.0);

  min_pos.push_back(0.8);
  min_pos.push_back(-0.3);
  min_pos.push_back(1.2);

  max_pos.push_back(0.9);
  max_pos.push_back(0.3);
  max_pos.push_back(1.4);

  /*w_vector[0] = {max_pos[0], max_pos[1], min_pos[2]};
   w_vector[1] = {min_pos[0], max_pos[1], min_pos[2]};
   w_vector[2] = {min_pos[0], min_pos[1], min_pos[2]};
   w_vector[3] = {max_pos[0], min_pos[1], min_pos[2]};
   w_vector[4] = {max_pos[0], max_pos[1], max_pos[2]};
   w_vector[5] = {min_pos[0], max_pos[1], max_pos[2]};
   w_vector[6] = {min_pos[0], min_pos[1], max_pos[2]};
   w_vector[7] = {max_pos[0], min_pos[1], max_pos[2]};*/

  //force contrl
  //ustaw pola, ktore sa ustawiane przez te funkcje
  //self.irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156)) #ustawienia do sterowania predkoscia
  error_pub = nh_.advertise<std_msgs::Float32>("uchyb", 1);
  image_info_subscriber = nh_.subscribe("realHomogMatrix", 1,
                                        &Irp6Control::image_info_callback,
                                        this);

}

void Irp6Control::image_info_callback(const serwo::SerwoInfo& msg) {
  T_gC_vector = msg.matrix;
  found = msg.found;
  vectorToTransform(T_gC_vector, found);
}

void Irp6Control::calculateNewVel() {
  for (int i = 0; i < 6; ++i) {
    if (error[i] > minError[i] || error[i] < -minError[i])
      newVel[i] = error[i] * p[i];
    else
      newVel[i] = 0;
  }
}

void Irp6Control::calculateNewCartesianPose() {
  //TODO current_pos_cartesian = self.irpos.get_cartesian_pose()
  std::vector<double> current_pos_cartesian;
  //atrapa
  current_pos_cartesian.push_back(0);
  current_pos_cartesian.push_back(1);
  current_pos_cartesian.push_back(2);
  new_pos_cartesian[0] = current_pos_cartesian[0] + newVel[0] * (1 / frequency);
  new_pos_cartesian[1] = current_pos_cartesian[1] + newVel[1] * (1 / frequency);
  new_pos_cartesian[2] = current_pos_cartesian[2] + newVel[2] * (1 / frequency);
}

void Irp6Control::calculateNewAcceleration() {
  for (int i = 0; i < 3; ++i)
    newAcceleration[i] = newVel[i] * frequency;
}

void Irp6Control::checkVelocityLimits() {
  for (int i = 0; i < 3; ++i) {
    if (newVel[i] > maxVel[i]) {
      newVel[i] = maxVel[i];
      //print 'Przekroczono limit predkosci w osi ' + str(i) + " : " + str(self.newVel[i])
    }
    if (newVel[i] < -maxVel[i]) {
      newVel[i] = -maxVel[i];
      //print 'Przekroczono limit predkosci w osi ' + str(i) + " : " + str(self.newVel[i])
    }
  }
}

void Irp6Control::checkAccelerationLimits() {
  for (int i = 0; i < 3; ++i) {
    if (!(newAcceleration[i] > -maxAcceleration[i]
        and newAcceleration[i] < maxAcceleration[i])) {
      //print 'Przekroczono limit przyspieszenia w osi ' + str(i) + ': ' + str(self.newAcceleration[i])
      newVel[0] = 0;
    }
  }
}

void Irp6Control::checkCartesianLimits() {
  for (int i = 0; i < 3; ++i)
    if (!(new_pos_cartesian[i] > min_pos[i]
        and new_pos_cartesian[i] < max_pos[i])) {
      //print 'Przekroczony limit pozycji w osi ' + str(i) + ': ' + str(self.new_pos_cartesian[i])
      newVel[i] = 0;
    }
}

void Irp6Control::setZeroValues() {
  newVel.clear();
  newVel.push_back(0);
  newVel.push_back(0);
  newVel.push_back(0);
  newVel.push_back(0);
  newVel.push_back(0);
  newVel.push_back(0);
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

void Irp6Control::regulatroDecomposition() {
  error[0] = T_gC[1][3];  //uchyb w osi x kamery
  error[1] = T_gC[0][3];  //uchyb w osi y kamery
  error[2] = T_gC[2][3];  //uchyb w osi z kamery
  //TODO dekopozycja katow os-kat
}

void Irp6Control::moveToJointPosition(std::vector<double> pos,
                                      double time_from_start) {

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_action_(
      "/irp6p_arm/spline_trajectory_action_joint", true);

  trajectory_action_.waitForServer();
  //std::cout<<trajectory_action_.isServerConnected()<<std::endl;

  control_msgs::FollowJointTrajectoryGoal jointGoal;

  jointGoal.trajectory.joint_names.push_back("joint1");
  jointGoal.trajectory.joint_names.push_back("joint2");
  jointGoal.trajectory.joint_names.push_back("joint3");
  jointGoal.trajectory.joint_names.push_back("joint4");
  jointGoal.trajectory.joint_names.push_back("joint5");
  jointGoal.trajectory.joint_names.push_back("joint6");

  //number of waypoints
  jointGoal.trajectory.points.resize(1);

  int ind = 0;
  jointGoal.trajectory.points[ind].positions.resize(6);

  jointGoal.trajectory.points[ind].positions[0] = pos[1];
  jointGoal.trajectory.points[ind].positions[1] = pos[2];
  jointGoal.trajectory.points[ind].positions[2] = pos[3];
  jointGoal.trajectory.points[ind].positions[3] = pos[4];
  jointGoal.trajectory.points[ind].positions[4] = pos[5];
  jointGoal.trajectory.points[ind].positions[5] = pos[6];

  jointGoal.trajectory.points[ind].time_from_start = ros::Duration(
      time_from_start);

  jointGoal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);

  trajectory_action_.sendGoal(jointGoal);

  trajectory_action_.waitForResult();

  //std::cout<<trajectory_action_.getState().toString();
  ROS_INFO("Sent Goal");
}

//void IrpControl::startForceControl(){

//}

//Change 16-elements vector to 4x4 matrix
void Irp6Control::vectorToTransform(cv::vector<double> v, int found) {
  if (found == 1) {
    tf::Vector3 translation_gC = tf::Vector3(v[3], v[7], v[11]);
    tf::Matrix3x3 rotation_gC = tf::Matrix3x3(v[0], v[1], v[2], v[4], v[5],
                                              v[6], v[8], v[9], v[10]);
    //T_gC = tf::Transform(rotation_gC, translation_gC);
  } else {
    tf::Vector3 translation_gC = tf::Vector3(0, 0, 0);
    tf::Matrix3x3 rotation_gC = tf::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
    // T_gC = tf::Transform(rotation_gC, translation_gC);
  }
  return;
}

void Irp6Control::run() {

  if (!T_gC_vector.empty()) {
    //T_gC = matrixOperations.translation_from_vector(T_gC_vector);//przeksztalcenie wektora na macierz
    regulatroDecomposition();
    calculateAndSetNewValues();
    //irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(self.newVel[0], self.newVel[1], self.newVel[2]), Vector3(self.newVel[3], self.newVel[4], self.newVel[5])));//sterowanie predkoscia
    //error_pub.publish(error[1]);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "serwovision_control");
  ros::NodeHandle nh;
  Irp6Control control(nh);

  std::vector<double> pos;

  //-------
  pos.push_back(0.0);
  pos.push_back(-1.57079632679);
  pos.push_back(0.0);
  pos.push_back(0.0);
  pos.push_back(4.71238898038);
  pos.push_back(1.57079632679);

  control.moveToJointPosition(pos, 10.0);
  ros::spinOnce();
  //-------
  /*while (ros::ok())
   {
   control.run();
   ros::spinOnce();
   }*/

  return 0;
}

