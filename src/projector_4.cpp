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
    const cv::Point2f src_pt[]={
             cv::Point2f(0.0, 0.0),
             cv::Point2f(1023.0 , 0.0),
             cv::Point2f(1023.0 , 767.0),
             cv::Point2f(0.0, 767.0)};
    //printf("x: %f y: %f", src_pt[2].x, src_pt[2].y);
    const cv::Point2f dst_pt[]={
             cv::Point2f(-620.0, 2660.0),
             cv::Point2f(620.0 , 2660.0),
             cv::Point2f(390.0 , 1320.0),
             cv::Point2f(-390.0, 1320.0)};
    //const cv::Mat homography_matrix = cv::getPerspectiveTransform(src_pt,dst_pt);
    //std::cout << "M = "<< std::endl << " "  << homography_matrix << std::endl << std::endl;
    cv::Point2f new_dst_pt[]={
             cv::Point2f(0.0, 0.0),
             cv::Point2f(0.0 , 0.0),
             cv::Point2f(0.0 , 0.0),
             cv::Point2f(0.0, 0.0)};
    ros::Rate rate(30);

    tf::TransformListener listener;
    while (ros::ok()) {

      n.getParam("exp_miki_img/switch", fin_switch);
      if (fin_switch == 0) {
        break;
      }
      //get pan degree
      tf::StampedTransform transform;
      try {
        listener.waitForTransform("/ud_base_footprint", "/ud_pt_plate_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("ud_base_footprint", "ud_pt_plate_link", ros::Time(0), transform);

      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      double current_pan = 0.9;
      const cv::Point2f rot[]={
               cv::Point2f(cos(current_pan), -sin(current_pan)),
               cv::Point2f(sin(current_pan), cos(current_pan))};
      for (int i = 0; i < 4; i++) {
        new_dst_pt[i].x = (rot[0].x * dst_pt[i].x) + (rot[0].y * dst_pt[i].y);
        new_dst_pt[i].y = (rot[1].x * dst_pt[i].x) + (rot[0].y * dst_pt[i].y);
        //new_dst_pt[i] = cv::Point2f(0.0, 0.0);
      }




      cv::imshow("screen_4", source_img);
      cv::waitKey(1);
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
