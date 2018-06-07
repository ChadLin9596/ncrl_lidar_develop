#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_normal");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
