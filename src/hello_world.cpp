#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hello_world");

  ros::NodeHandle nh;

  ros::Rate rate(1);

  while(ros::ok()){
    ROS_INFO_STREAM("hello ROS world !!!");
    rate.sleep();
  }

  return 0
}
