#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <serwo/SerwoInfo.h>

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

  //1/0 depends on diod grid grid was found / not found
  int found;

  //-----------------------------------
  //mesage to be published (grid position in camera frame)
  serwo::SerwoInfo msg;
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
  //int modelMask(std::vector<std::vector<int>> ,cv::Point2f, int, cv::Mat);

};
/*
 std::vector<std::vector<int>> mask - maska modelu, macierz 2x2 z modelem, który chcemy dopasować do obrazu
 cv::Point2f wsp_max - punkt wstępnie uznany za środek diody, od niego zaczniemy poszukiwania
 int range - rozmiar macierzy otaczającej piksel uznany za środkowy
 cv::Mat image - przetarzany obraz


int ImageProcessing::modelMask(std::vector<std::vector<int>> mask,cv::Point2f& wsp_max, int range, cv::Mat image){
    cv::Point2f& wsp_max_new,
    //sprawdzanie czy badana macierz otaczająca piksel uznany za środek diody nie wykracza poza macierz całego obrazu
    bool in_range = false;
    int rows = image.size().height;
    int cols = image.size().width;
    if (wsp_maxB.y - range > 0 && wsp_maxB.y + range < rows && wsp_maxB.x - range > 0 && wsp_maxB.x + range < cols)
        in_range = true;
    else
        return 0;

    double match_wsp = 0.0; //wspolczynnik dopasowania, ktory chcemy maksymalizować
    double tmp = 0.0;
    int b = 0;
    int g = 0;
    int r = 0;

    uchar * ptr;
    for (int i = wsp_maxB.y - range; i < wsp_maxB.y + range - mask.size(); ++i) {
      for (int j = wsp_maxB.x - range; j < wsp_maxB.x + range -mask.size(); ++j)
          for (int k = 0; k < range; ++k)
              for (int l = 0; l < range; ++l){
                ptr = image.ptr(i+k);
                b = ptr[3 * j+l];
                g = ptr[3 * j+l + 1];
                r = ptr[3 * j+l + 2];
                tmp += mask(k,l)*(b+g+r);
              }
      if (tmp > match_wsp){
          match_wsp = tmp;

      }




    return 1;
}
*/

void ImageProcessing::callback(const sensor_msgs::ImageConstPtr& img) {
  sourceImage = cv_bridge::toCvShare(img, "bgr8")->image.clone();
}

std::vector<cv::Point2f> ImageProcessing::Generate2DPoints() {
  cv::Mat image = sourceImage;

  double sumaB = 0;
  double sumaG = 0;
  double sumaR = 0;

  //wsp maksymalnych składowych
  cv::Point2f wsp_maxB;
  cv::Point2f wsp_maxG;
  cv::Point2f wsp_maxR;
  cv::Point2f wsp_maxW;

  //maksymalne składowe
  int maxB = 0;
  double maxBminG = 255;
  double maxBminR = 255;

  int maxG = 0;
  double maxGminB = 255;
  double maxGminR = 255;

  int maxR = 0;
  double maxRminB = 255;
  double maxRminG = 255;

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

  double licznik = 0.0;
  double suma_i = 0.0;
  double suma_j = 0.0;
  int i = 0;
  int j = 0;

  if (wsp_maxB.y - 6 > 0 && wsp_maxB.y + 6 < rows && wsp_maxB.x - 6 > 0 && wsp_maxB.x + 6 < cols){
      for (i = wsp_maxB.y - 2; i < wsp_maxB.y + 2; ++i) {
          ptr = image.ptr(i);
          for (j = wsp_maxB.x - 2; j < wsp_maxB.x + 2; ++j) {
              /*if ((ptr[3*j] + ptr[3*j+1] + ptr[3*j+2])> 10){
                  licznik += (ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]);
                  suma_i += i*((ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]));
                  suma_j += j*(ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]);
              }*/
              if (ptr[3*j+2] > maxG){
                  maxG = ptr[3*j+2];
                  wsp_maxB.y = i;
                  wsp_maxB.x = j;
              }
          }
      }
      //wsp_maxB.y = (float)(suma_i/licznik);
      //wsp_maxB.x = (float)(suma_j/licznik);
  }

  licznik = 0.0;
  suma_i = 0.0;
  suma_j = 0.0;
  if (wsp_maxG.y - 6 > 0 && wsp_maxG.y + 6 < rows && wsp_maxG.x - 6 > 0 && wsp_maxG.x + 6 < cols){
      for (i = wsp_maxG.y - 6; i < wsp_maxG.y + 6; ++i) {
          ptr = image.ptr(i);
          for (j = wsp_maxG.x - 6; j < wsp_maxG.x + 6; ++j) {
              /*if ((ptr[3*j] + ptr[3*j+1] + ptr[3*j+2])> 10){
                  licznik += (ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]);
                  suma_i += i*((ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]));
                  suma_j += j*(ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]);
              }*/
              if (ptr[3*j+2] > maxG){
                  maxG = ptr[3*j+2];
                  wsp_maxG.y = i;
                  wsp_maxG.x = j;
              }
          }
      }
      //wsp_maxG.y = (float)(suma_i/licznik);
      //wsp_maxG.x = (float)(suma_j/licznik);
  }

  licznik = 0.0;
  suma_i = 0.0;
  suma_j = 0.0;
  if (wsp_maxW.y - 6 > 0 && wsp_maxW.y + 6 < rows && wsp_maxW.x - 6 > 0 && wsp_maxW.x + 6 < cols){
      for (i = wsp_maxW.y - 1; i < wsp_maxW.y + 1; ++i) {
          ptr = image.ptr(i);
          for (j = wsp_maxW.x - 1; j < wsp_maxW.x + 1; ++j) {
              if ((ptr[3*j] + ptr[3*j+1] + ptr[3*j+2])> 10){
                  licznik += (ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]);
                  suma_i += i*((ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]));
                  suma_j += j*(ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]);
              }
          }
      }
      wsp_maxW.y = (int)(suma_i/licznik);
      wsp_maxW.x = (int)(suma_j/licznik);
  }

  licznik = 0.0;
  suma_i = 0.0;
  suma_j = 0.0;
  if (wsp_maxR.y - 6 > 0 && wsp_maxR.y + 6 < rows && wsp_maxR.x - 6 > 0 && wsp_maxR.x + 6 < cols){
      for (i = wsp_maxR.y - 6; i < wsp_maxR.y + 6; ++i) {
          ptr = image.ptr(i);
          for (j = wsp_maxR.x - 6; j < wsp_maxR.x + 6; ++j) {
              /*if ((ptr[3*j] + ptr[3*j+1] + ptr[3*j+2])> 10){
                  licznik += (ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]);
                  suma_i += i*((ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]));
                  suma_j += j*(ptr[3*j] + ptr[3*j+1] + ptr[3*j+2]);
              }*/
              if (ptr[3*j+2] > maxG){
                  maxG = ptr[3*j+2];
                  wsp_maxR.y = i;
                  wsp_maxR.x = j;
              }
          }
      }
      //if (wsp_maxR.x != 638 && wsp_maxR.y != 444)
      //    std::cout<<"aa"<<std::endl;

      //wsp_maxR.y = (float)(suma_i/licznik);
      //wsp_maxR.x = (float)(suma_j/licznik);
  }

  std::vector<cv::Point2f> gridPoints;
  found = 1;
  gridPoints.push_back(wsp_maxB);
  gridPoints.push_back(wsp_maxG);
  gridPoints.push_back(wsp_maxR);
  gridPoints.push_back(wsp_maxW);
  //std::cout<<"B"<<std::endl;
  //std::cout<<wsp_maxB<<std::endl;
  //std::cout<<"G"<<std::endl;
  //std::cout<<wsp_maxG<<std::endl;
  //std::cout<<"W"<<std::endl;
  std::cout<<wsp_maxW<<std::endl;
  //std::cout<<"R"<<std::endl;
  //std::cout<<wsp_maxR<<std::endl;
  //std::cout<<maxG<<std::endl;
  //std::cout<<""<<std::endl;
  return gridPoints;
}

ImageProcessing::ImageProcessing(ros::NodeHandle &nh) {
  nh_ = nh;
  imageSubscriber = nh_.subscribe("/camera/image_color", 1000,
                                  &ImageProcessing::callback, this);

  grid_info_pub = nh_.advertise<serwo::SerwoInfo>("object_seen_by_camera", 1);

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

  static char * tmp = NULL;
  static int tmpi;
  ros::init(tmpi, &tmp, "image_processing", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ImageProcessing imageProcessing(nh);

  while (ros::ok()) {
    //updating image points
    imageProcessing.imagePoints = imageProcessing.Generate2DPoints();
    //solvePnP
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

    /*printf("[ %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f]\n ", rotationMatrix(0,0), rotationMatrix(0,1), rotationMatrix(0,2), imageProcessing.tvec(0),
     rotationMatrix(1,0), rotationMatrix(1,1), rotationMatrix(1,2), imageProcessing.tvec(1),
     rotationMatrix(2,0), rotationMatrix(2,1), rotationMatrix(2,2), imageProcessing.tvec(2),
     0.0, 0.0, 0.0, 1.0);*/
    //push pattern_pose to vector
    std::vector<double> pattern_pose_vector;
    pattern_pose_vector.push_back(rotationMatrix(0, 0));
    pattern_pose_vector.push_back(rotationMatrix(0, 1));
    pattern_pose_vector.push_back(rotationMatrix(0, 2));
    pattern_pose_vector.push_back(imageProcessing.tvec(0));
    pattern_pose_vector.push_back(rotationMatrix(1, 0));
    pattern_pose_vector.push_back(rotationMatrix(1, 1));
    pattern_pose_vector.push_back(rotationMatrix(1, 2));
    pattern_pose_vector.push_back(imageProcessing.tvec(1));
    pattern_pose_vector.push_back(rotationMatrix(2, 0));
    pattern_pose_vector.push_back(rotationMatrix(2, 1));
    pattern_pose_vector.push_back(rotationMatrix(2, 2));
    pattern_pose_vector.push_back(imageProcessing.tvec(2));
    pattern_pose_vector.push_back(0.0);
    pattern_pose_vector.push_back(0.0);
    pattern_pose_vector.push_back(0.0);
    pattern_pose_vector.push_back(1.0);

    //std::fstream plik;
    //plik.open( "/home/mwegiere/DC.txt", std::ios::app | std::ios::out);
    //for (int i=0; i<16; ++i)
    //std::cout<<pattern_pose_vector[3]<<std::endl;
    //plik.close();
    //create message
    imageProcessing.msg.matrix = pattern_pose_vector;
    imageProcessing.msg.found = imageProcessing.found;
    imageProcessing.msg.out_time_nsec_pocz = 0;
    imageProcessing.msg.out_time_sec_pocz = 0;
    imageProcessing.msg.out_time_nsec_kon = 0;
    imageProcessing.msg.out_time_sec_kon = 0;
    //publish message
    imageProcessing.grid_info_pub.publish(imageProcessing.msg);

    ros::spinOnce();
  }
  return 0;
}
