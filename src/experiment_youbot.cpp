#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "map_ud_base_footprint_listener");
  ros::NodeHandle node;
  ros::Rate rate(10.0);

  tf::TransformListener listener;
  tf::StampedTransform transform;

  while(node.ok()) {
    try {
      listener.lookupTransform("map",
                   "ud_base_footprint",
                   ros::Time(0),
                   transform);
    }
    catch(tf::TransformException &exception) {
      ROS_ERROR("%s", exception.what());
    }


    std::cout << "T = "<< std::endl << " "  << transform.getOrigin() << std::endl << std::endl;
    std::cout << "R = "<< std::endl << " "  << transform.getRotation() << std::endl << std::endl;



    rate.sleep();
  }

  return 0;
}
