#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    std::string file_path = ros::package::getPath("experiment_miki") + "/image/";
    cv::Mat source_image = cv::imread(file_path + "pop.png", cv::IMREAD_GRAYSCALE);
    cv::imshow("image", source_image);
    cv::waitKey();

    cv::imwrite(file_path + "capture.jpg", source_image);
    return 0;
}
