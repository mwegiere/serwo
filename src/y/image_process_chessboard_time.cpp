#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <serwo/ErrorInfo.h>

#include <iostream>
//#include <chrono>

#include <fstream>

class ImageProcessing {
 public:
  ImageProcessing(ros::NodeHandle &nh);
  //-----------------------------------
  //++++input of SolvePnP++++
  //generating camera image points
  std::vector<cv::Point2f> Generate2DPoints();
  //generating real object points
  std::vector<cv::Point3f> Generate3DPoints();
  //camera image points
  std::vector<cv::Point2f> imagePoints;
  //real object points
  std::vector<cv::Point3f> objectPoints;
  //distort coefficients
  cv::Mat distCoeffs;
  //camera matrix
  cv::Mat cameraMatrix;
  //-----------------------------------
  //++++output of SolvePnP++++
  //rotation vector
  cv::Mat_<double> rvec;
  //translation vector
  cv::Mat_<double> tvec;
  //-----------------------------------

  //1/0 depends on diod grid was found / not found
  int found;

  //-----------------------------------
  //mesage to be published (grid position in camera frame)
  serwo::ErrorInfo msg;
  //publisher (grid position in camera frame)
  ros::Publisher grid_info_pub;

  cv::Size patternSize;

  cv::Mat sourceImage;
 private:
  //original image from camera
  
  ros::NodeHandle nh_;
  //subscribe an image from camera topic
  ros::Subscriber imageSubscriber;
  //callback function used for subscribe image from topic
  void callback(const sensor_msgs::ImageConstPtr& img);
};

void ImageProcessing::callback(const sensor_msgs::ImageConstPtr& img) {
  sourceImage = cv_bridge::toCvShare(img, "bgr8")->image.clone();
}

std::vector<cv::Point2f> ImageProcessing::Generate2DPoints() {

  cv::Mat image = sourceImage;
  cv::Mat out;
  std::cout<<"aa"<<std::endl;
  //cv::cvtColor(image, out, CV_RGB2GRAY);

  //patternSize = cvSize(5,4);

  std::vector<cv::Point2f> imagePoints2;
  cv::findChessboardCorners(out, cvSize(5,4), imagePoints2, 1);

  return imagePoints2;
}

ImageProcessing::ImageProcessing(ros::NodeHandle &nh) {

  nh_ = nh;
  imageSubscriber = nh_.subscribe("/camera/image_color", 1000,
                                  &ImageProcessing::callback, this);
  grid_info_pub = nh_.advertise<serwo::ErrorInfo>("/error", 1);

  //3D model points are constant
  objectPoints = Generate3DPoints();

  distCoeffs = cv::Mat(5, 1, cv::DataType<double>::type);
  distCoeffs.at<double>(0) = -0.413271;
  distCoeffs.at<double>(1) = 0.172960;
  distCoeffs.at<double>(2) = -0.004729;
  distCoeffs.at<double>(3) = -0.000895;
  distCoeffs.at<double>(4) = 0.0;

  cameraMatrix = cv::Mat(3, 3, cv::DataType<double>::type);
  cameraMatrix.at<double>(0, 0) = 1080.054285;
  cameraMatrix.at<double>(0, 1) = 0.0;
  cameraMatrix.at<double>(0, 2) = 655.287407;
  cameraMatrix.at<double>(1, 0) = 0.0;
  cameraMatrix.at<double>(1, 1) = 1072.245647;
  cameraMatrix.at<double>(1, 2) = 524.778361;
  cameraMatrix.at<double>(2, 0) = 0.0;
  cameraMatrix.at<double>(2, 1) = 0.0;
  cameraMatrix.at<double>(2, 2) = 1.0;

  rvec = cv::Mat(3, 1, cv::DataType<double>::type);
  tvec = cv::Mat(3, 1, cv::DataType<double>::type);
}

std::vector<cv::Point3f> ImageProcessing::Generate3DPoints() {

  int prop_width = 5;
  int prop_height = 4;
  int prop_square_width = 0.018333;
  int prop_square_height = 0.01852;

  std::vector<cv::Point3f> modelPoints;
  for (int i = 0; i < prop_height; ++i) {
      for (int j = 0; j < prop_width; ++j) {
          modelPoints.push_back(cv::Point3f(-j * prop_square_height, -i * prop_square_width, 0));
      }
  }
  return modelPoints;
}

int main(int argc, char **argv) {

   //std::ofstream pnp_time;
   //pnp_time.open ("pnp_time.txt");
   //std::ofstream recognition_time;
   //recognition_time.open ("recognition_time.txt");
  static char * tmp = NULL;
  static int tmpi;
  ros::init(tmpi, &tmp, "image_processing", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  ImageProcessing imageProcessing(nh);
  ros::Rate loop_rate(10);
  loop_rate.sleep();

  while (imageProcessing.sourceImage.size().height == 0){
        ros::spinOnce();
	loop_rate.sleep();
  }
 std::cout<<imageProcessing.sourceImage.size().height<<std::endl;

//  /ros::spinOnce();
  loop_rate.sleep();
  while (ros::ok()) {

    //updating image points
    //auto start_recognition= std::chrono::high_resolution_clock::now();
    //imageProcessing.imagePoints = imageProcessing.Generate2DPoints();

    cv::Mat image = imageProcessing.sourceImage;
    cv::Mat out;

    cv::cvtColor(image, out, CV_RGB2GRAY);
    std::cout<<out.size().height<<std::endl;
    
    bool a = cv::findChessboardCorners(out, cvSize(5,4), imageProcessing.imagePoints);

    std::cout<<a<<std::endl;

    //auto finish_recognition = std::chrono::high_resolution_clock::now();
    //recognition_time << std::chrono::duration_cast<std::chrono::nanoseconds>(finish_recognition-start_recognition).count()<<"\n";

    //solvePnP
    //auto start_pnp = std::chrono::high_resolution_clock::now();

    cv::solvePnP(imageProcessing.objectPoints, imageProcessing.imagePoints,
                 imageProcessing.cameraMatrix, imageProcessing.distCoeffs,
                 imageProcessing.rvec, imageProcessing.tvec);

    //change rvec to matrix
    cv::Mat_<double> rotationMatrix;
    cv::Rodrigues(imageProcessing.rvec, rotationMatrix);
    //final matrix (grid position in camera frame)
    cv::Mat pattern_pose =
        (cv::Mat_<double>(4, 4) << rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(
            0, 2), imageProcessing.tvec(0), rotationMatrix(1, 0), rotationMatrix(
            1, 1), rotationMatrix(1, 2), imageProcessing.tvec(1), rotationMatrix(
            2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2), imageProcessing
            .tvec(2), 0, 0, 0, 1);

    imageProcessing.msg.error = imageProcessing.tvec(0);
    imageProcessing.msg.found = 1;
    //imageProcessing.msg.found = imageProcessing.found;

    //publish message
    imageProcessing.grid_info_pub.publish(imageProcessing.msg);

    //auto finish_pnp = std::chrono::high_resolution_clock::now();
    //pnp_time << std::chrono::duration_cast<std::chrono::nanoseconds>(finish_pnp-start_pnp).count()<<"\n";

    ros::spinOnce();
  }
  //pnp_time.close();
  //recognition_time.close();
  return 0;
}
