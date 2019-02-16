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
  ros::NodeHandle n;

  int exp_num = 0;
  int fin_switch = 1;
  n.getParam("/exp_num", exp_num);
  n.setParam("exp_miki_img/switch", 1);

  if (exp_num == 4) {
    std::string file_dir = ros::package::getPath("experiment_miki") + "/src/image/";
    std::string input_file_path = file_dir + "pop_90.png";
    cv::Mat source_img = cv::imread(input_file_path, cv::IMREAD_UNCHANGED);

    ros::Rate rate(30);
    while (ros::ok()) {
      const cv::Point2f src_pt[]={
               cv::Point2f(0.0, 0.0),
               cv::Point2f(1023.0 , 0.0),
               cv::Point2f(1023.0 , 767.0),
               cv::Point2f(0.0, 767.0)};
      printf("x: %f y: %f", src_pt[2].x, src_pt[2].y);
      cv::imshow("screen_4", source_img);
      cv::waitKey(1);
      n.getParam("exp_miki_img/switch", fin_switch);
      if (fin_switch == 0) {
        break;
      }
      rate.sleep();
    }
    cv::destroyWindow("screen_4");
  }


}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "exp_4_cpp");

  ros::NodeHandle n;


  n.setParam("/exp_num", 1);
  ros::Subscriber sub = n.subscribe("finish_pantilt", 1000, &Callback);

  ros::Rate rate(30);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
    }


  return 0;
}
