#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
  std::string file_dir = ros::package::getPath("experiment_miki") + "/src/image/";
  std::string input_file_path = file_dir + "pop.png";
  cv::Mat source_img = cv::imread(input_file_path, cv::IMREAD_GRAYSCALE);
  cv::imshow("image", source_img);
  cv::waitKey();
  std::string output_file_path = file_dir + "capture.jpg";
  cv::imwrite(output_file_path, source_img);
  return 0;
}
