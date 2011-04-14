#include <ros/ros.h>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "test_get_param");

  ros::NodeHandle nh;
  nh.setParam(std::string("monkey"), false);
  bool test_bool;
  while(ros::ok()) {
    if(!nh.getParam("monkey", test_bool)) {
      ROS_INFO_STREAM("Failed, bailing");
      ros::shutdown();
    }
    std::cout << ".";
  }
  return 0;
}
