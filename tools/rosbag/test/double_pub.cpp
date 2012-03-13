#include <ros/ros.h>
#include <std_msgs/String.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "double_pub");
  ros::NodeHandle n;
  ros::Publisher p1 = n.advertise<std_msgs::String>("chatter", 1);
  ros::Publisher p2 = n.advertise<std_msgs::String>("rettahc", 1);

  ros::Rate r(10.0);
  while(ros::ok())
  {
    std_msgs::String s;
    s.data = "hello";
    p1.publish(s);
    s.data = "goodbye";
    p2.publish(s);
  }
  return 0;
}
