#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int data_base = 0;

void Callback(const std_msgs::Int16& msg)
{
  //std::cout << msg.data << std::endl;
  data_base=msg.data;

  std::string file_dir = ros::package::getPath("experiment_miki") + "/src/image/";
  std::string input_file_path = file_dir + "pop_90.png";
  cv::Mat source_img = cv::imread(input_file_path, cv::IMREAD_UNCHANGED);
  cv::imshow("image", source_img);
  cv::waitKey();

  return 0;

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "exp_4_cpp");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("finish_pantilt", 1000, Callback);

  ros::Rate rate(30);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
    }


  return 0;
}
