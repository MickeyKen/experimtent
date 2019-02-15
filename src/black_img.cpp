#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "black_image");
  ros::NodeHandle nh;

  nh.setParam("black_img/switch", 1);

  int black_image_switch = 1;
  
  cv::Mat black_img = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3); 
  //cv::namedWindow("black image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cvNamedWindow("black_image", CV_WINDOW_NORMAL);
  cvSetWindowProperty("black_image", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

  cv::imshow("black_image", black_img);

  cv::waitKey(10);
  
  while (1) {
    nh.getParam("black_img/switch", black_image_switch);
    if (black_image_switch == 0) 
      break;
  }

  cv::destroyWindow("black_image");

  return 0;
}
