#include <ros/node.h>
#include <ros/publisher.h>
#include <roslib/Time.h>

#include <boost/thread.hpp>

#include <sstream>


roslib::Time g_time;
void incomingTime()
{
}

void sendThread(std::string topic)
{
  while (1)
  {
    roslib::Time m;
    ros::Node::instance()->publish(topic, m);

    usleep(1);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv);

  ros::Node n("blah", ros::Node::DONT_HANDLE_SIGINT);

  for (int i = 0; i < 100; ++i)
  {
    std::stringstream ss;
    ss << "time" << i;

    n.subscribe(ss.str(), g_time, incomingTime, 1);
    n.advertise<roslib::Time>(ss.str(), 1);
    boost::thread t(boost::bind(sendThread, ss.str()));

    printf("%s\n", ss.str().c_str());
  }

  n.spin();

  return 0;
}
