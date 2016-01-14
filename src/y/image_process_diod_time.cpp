#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <serwo/ErrorInfo.h>

#include <iostream>
#include <chrono>

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

 private:
  //original image from camera
  cv::Mat sourceImage;
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

  //wsp maksymalnych składowych
  cv::Point2f wsp_maxB;
  cv::Point2f wsp_maxG;
  cv::Point2f wsp_maxR;
  cv::Point2f wsp_maxW;

  wsp_maxB.x = 0;
  wsp_maxB.y = 0;
  wsp_maxG.x = 0;
  wsp_maxG.y = 0;
  wsp_maxR.x = 0;
  wsp_maxR.y = 0;
  wsp_maxW.x = 0;
  wsp_maxW.y = 0;

  int maxB = 0;
  int maxG = 0;
  int maxR = 0;
  int maxW = 0;

  int rows = image.size().height;
  int cols = image.size().width;

  uchar * ptr;
  for (int i = 3; i < rows; ++i) {
    ptr = image.ptr(i);
    for (int j = 3; j < cols; ++j) {
      int b = ptr[3 * j];
      int g = ptr[3 * j + 1];
      int r = ptr[3 * j + 2];

      if (b - (r + g) > maxB) {
        wsp_maxB.y = i;
        wsp_maxB.x = j;
        maxB = b - (r + g);
      }

      if (g - (r + b) > maxG) {
        wsp_maxG.y = i;
        wsp_maxG.x = j;
        maxG = g - (r + b);
      }

      if (r - (g + b) > maxR) {
        wsp_maxR.y = i;
        wsp_maxR.x = j;
        maxR = r - (g + b);
      }

      if (r + g + b > maxW) {
        wsp_maxW.y = i;
        wsp_maxW.x = j;
        maxW = r + g + b;
      }
    }
  }

  cv::Point2f wsp_maxB_new;
  cv::Point2f wsp_maxG_new;
  cv::Point2f wsp_maxR_new;
  cv::Point2f wsp_maxW_new;

  int i = 0;
  int j = 0;
  maxB = 0;
  maxG = 0;
  if (wsp_maxB.y - 3 > 0 && wsp_maxB.y + 3 < rows && wsp_maxB.x - 3 > 0 && wsp_maxB.x + 3 < cols){
      for (i = wsp_maxB.y - 3; i < wsp_maxB.y + 3; ++i) {
          ptr = image.ptr(i);
          for (j = wsp_maxB.x - 3; j < wsp_maxB.x + 3; ++j) {
              if (ptr[3*j] > maxB){
                  maxB = ptr[3*j];
                  wsp_maxB_new.y = i;
                  wsp_maxB_new.x = j;
              }
          }
      }
  }

  maxB = 0;
  maxG = 0;
  if (wsp_maxG.y - 3 > 0 && wsp_maxG.y + 3 < rows && wsp_maxG.x - 3 > 0 && wsp_maxG.x + 3 < cols){
      for (i = wsp_maxG.y - 3; i < wsp_maxG.y + 3; ++i) {
          ptr = image.ptr(i);
          for (j = wsp_maxG.x - 3; j < wsp_maxG.x + 3; ++j) {
              if (ptr[3*j+1] > maxG){
                  maxG = ptr[3*j+1];
                  wsp_maxG_new.y = i;
                  wsp_maxG_new.x = j;
              }
          }
      }
  }

  maxB = 0;
  maxG = 0;
  if (wsp_maxR.y - 3 > 0 && wsp_maxR.y + 3 < rows && wsp_maxR.x - 3 > 0 && wsp_maxR.x + 3 < cols){
      for (i = wsp_maxR.y - 3; i < wsp_maxR.y + 3; ++i) {
          ptr = image.ptr(i);
          for (j = wsp_maxR.x - 3; j < wsp_maxR.x + 3; ++j) {
              if (ptr[3*j+2] > maxG){
                  maxG = ptr[3*j+2];
                  wsp_maxR_new.y = i;
                  wsp_maxR_new.x = j;
              }
          }
      }
  }

  //int licznik = 0;
  //int suma_i = 0;
  //int suma_j = 0;
  maxW = 0;
  if (wsp_maxW.y - 3 > 0 && wsp_maxW.y + 3 < rows && wsp_maxW.x - 3 > 0 && wsp_maxW.x + 3 < cols){
      for (i = wsp_maxW.y - 3; i < wsp_maxW.y + 3; ++i) {
          ptr = image.ptr(i);
          for (j = wsp_maxW.x - 3; j < wsp_maxW.x + 3; ++j) {
                  if (ptr[3*j] + ptr[3*j+1]  > maxW){
                      maxW = ptr[3*j] + ptr[3*j+1];
                      wsp_maxW_new.y = i;
                      wsp_maxW_new.x = j;
                  }
          }
      }
  }

  std::vector<cv::Point2f> gridPoints;

  if(wsp_maxB_new.x != 0 && wsp_maxB_new.y != 0 && wsp_maxG_new.x != 0 && wsp_maxG_new.y != 0 && wsp_maxR_new.x != 0 && wsp_maxR_new.y != 0 && wsp_maxW_new.x != 0 && wsp_maxW_new.y != 0){
      found = 1;
  }
  else{
      found = 0;
  }

  gridPoints.push_back(wsp_maxB_new);
  gridPoints.push_back(wsp_maxG_new);
  gridPoints.push_back(wsp_maxR_new);
  gridPoints.push_back(wsp_maxW_new);

  /*std::cout<<"B"<<std::endl;
  std::cout<<wsp_maxB<<std::endl;
  std::cout<<"G"<<std::endl;
  std::cout<<wsp_maxG<<std::endl;
  std::cout<<"W"<<std::endl;
  std::cout<<wsp_maxW<<std::endl;
  std::cout<<"R"<<std::endl;
  std::cout<<wsp_maxR<<std::endl;
  std::cout<<maxG<<std::endl;
  std::cout<<""<<std::endl;*/
  return gridPoints;
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
  std::vector<cv::Point3f> points;
  points.push_back(cv::Point3f(0.04813, 0.00802, 0));  //B
  points.push_back(cv::Point3f(0.00943, 0.0457, 0));  //G
  points.push_back(cv::Point3f(0.090827, 0.04535, 0));  //R
  points.push_back(cv::Point3f(0.09066, 0.004626, 0));  //W
  return points;
}

int main(int argc, char **argv) {

   std::ofstream pnp_time;
   pnp_time.open ("pnp_time.txt");
   std::ofstream recognition_time;
   recognition_time.open ("recognition_time.txt");


  static char * tmp = NULL;
  static int tmpi;
  ros::init(tmpi, &tmp, "image_processing", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ImageProcessing imageProcessing(nh);

  while (ros::ok() ){
    //updating image points
    auto start_recognition= std::chrono::high_resolution_clock::now();
    imageProcessing.imagePoints = imageProcessing.Generate2DPoints();
    auto finish_recognition = std::chrono::high_resolution_clock::now();
    recognition_time << std::chrono::duration_cast<std::chrono::nanoseconds>(finish_recognition-start_recognition).count()<<"\n";

    //solvePnP
    auto start_pnp = std::chrono::high_resolution_clock::now();

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
    imageProcessing.msg.found = imageProcessing.found;
    //publish message
    imageProcessing.grid_info_pub.publish(imageProcessing.msg);

    auto finish_pnp = std::chrono::high_resolution_clock::now();
    pnp_time << std::chrono::duration_cast<std::chrono::nanoseconds>(finish_pnp-start_pnp).count()<<"\n";
    ros::spinOnce();
  }
  pnp_time.close();
  recognition_time.close();
  return 0;
}
