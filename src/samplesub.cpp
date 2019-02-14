#include "ros/ros.h"
#include <stdio.h>
#include <iostream>

void chatterCallback(const image_tutorial::image msg)
{
  cout << "height = " << msg.img.height <<
          " width = " << msg.img.width <<
          " frameID = " << msg.frameID << endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "subscriber");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("image_data", 1000, chatterCallback);

  ros::spin();

  return 0;
}

