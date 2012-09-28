#include <cstdlib>
#include <gtest/gtest.h>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <ros/console.h>
#include <ros/poll_manager.h>

bool dummyService(std_srvs::Empty::Request &req,std_srvs::Empty::Request &res)
{

  return true;
}

static const char SERVICE1[] = "service1";

void call(ros::ServiceClient &client)
{

  if (client && client.exists() && client.isValid())
  {
    //    ROS_INFO("Calling service");
    std_srvs::Empty srv;
    client.call(srv); // these will alternate successful and failed.
  }
  //  else
  //    ROS_INFO("Persistent client is invalid");
}

// this only verifies that it doesn't deadlock.  Should run about 60 seconds.
TEST(roscpp, ServiceDeadlocking)
{
  ros::ServiceClient client;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  unsigned j=0;
  ros::Time start_time = ros::Time::now();
  unsigned seconds = 30;
  ros::Time stop_time = start_time + ros::Duration(seconds, 0);

  while (true)
  {
    if ((j % 100 == 0) && (ros::Time::now() > stop_time))
      break;

    {
      ros::NodeHandle n2;
      ros::ServiceServer service = n2.advertiseService(SERVICE1, dummyService);
      client  = n2.serviceClient<std_srvs::Empty>(SERVICE1, true);
      call(client);
      service.shutdown();
    }
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService(SERVICE1, dummyService);

    call(client);
    ++j;
  }
  ROS_INFO("Made it through %u loops in %u seconds", j, seconds);
  ASSERT_GE(j, 1000);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "service_deadlock");
  return RUN_ALL_TESTS();
}
