#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_datatypes.h>

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

  if (exp_num == 21) {
    std::string file_dir = ros::package::getPath("experiment_miki") + "/src/image/";
    std::string input_file_path = file_dir + "pop_90.png";
    cv::Mat source_img = cv::imread(input_file_path, cv::IMREAD_UNCHANGED);
    int ColumnOfNewImage = 1024;
    int RowsOfNewImage = 768;
    resize(source_img, source_img, cv::Size(ColumnOfNewImage,RowsOfNewImage));

    // cv::namedWindow( "screen_4", CV_WINDOW_NORMAL );
    // cv::setWindowProperty("screen_4",CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);

    const cv::Point2f src_pt[]={
             cv::Point2f(0.0, 0.0),
             cv::Point2f(1023.0 , 0.0),
             cv::Point2f(1023.0 , 767.0),
             cv::Point2f(0.0, 767.0)};

    //const cv::Mat homography_matrix = cv::getPerspectiveTransform(src_pt,dst_pt);
    //std::cout << "M = "<< std::endl << " "  << homography_matrix << std::endl << std::endl;
    cv::Point2f new_dst_pt[]={
             cv::Point2f(0.0, 0.0),
             cv::Point2f(0.0 , 0.0),
             cv::Point2f(0.0 , 0.0),
             cv::Point2f(0.0, 0.0)};

    // CV_32F is float
    cv::Mat rot_x(3, 3, CV_32F);
    cv::Mat rot_y(3, 3, CV_32F);
    cv::Mat rot_z(3, 3, CV_32F);

    cv::Mat target = (cv::Mat_<double>(4,3) << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

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
        listener.waitForTransform("/ud_base_footprint", "/ud_pt_projector_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/ud_base_footprint", "/ud_pt_projector_link", ros::Time(0), transform);

      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      tf::Quaternion q(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

      double current_pan = yaw;



      //printf("\n");
      cv::Mat warp_img(cv::Size(1024, 768), CV_8U, cv::Scalar::all(0));
      cv::Mat M = cv::getPerspectiveTransform(src_pt,new_dst_pt);
      cv::warpPerspective( source_img, warp_img, M, source_img.size());
      // std::cout << "g = "<< std::endl << " "  << M << std::endl << std::endl;
      cv::imshow("screen_4", warp_img);
      cv::waitKey(1);
      //printf("finish");
      rate.sleep();
    }


    cv::destroyWindow("screen_4");
  }


}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "exp_21_cpp");

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
