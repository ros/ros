#include <cstdlib>
#include <gtest/gtest.h>
#include <list>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <log4cxx/appenderskeleton.h>
#include <ros/console.h>
#include <ros/poll_manager.h>

class ListAppender : public log4cxx::AppenderSkeleton
{
public:
  void append(const log4cxx::spi::LoggingEventPtr& event, log4cxx::helpers::Pool&)
  {
    list.push_back(event);
  }

  void close()
  {
    this->closed = true;
  }

  bool isClosed() const
  {
    return closed;
  }

  bool requiresLayout() const
  {
    return false;
  }
  
  const std::list<log4cxx::spi::LoggingEventPtr>& getList() const
  {
    return list;
  }

protected:
  std::list<log4cxx::spi::LoggingEventPtr> list;
};
typedef log4cxx::helpers::ObjectPtrT<ListAppender> ListAppenderPtr;

static const char EXCEPTION[] = "custom exception message";

bool throwingService(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
  throw std::runtime_error(EXCEPTION);
  return true;
}

static const char SERVICE[] = "service_exception";

TEST(roscpp, ServiceThrowingException)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService(SERVICE, throwingService);

  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("ros.roscpp");
  ListAppenderPtr appender = new ListAppender();
  logger->addAppender(appender);

  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>(SERVICE, true);
  std_srvs::Empty srv;
  bool success = client.call(srv);
  ASSERT_FALSE(success);

  bool found_error_output = false;
  const std::list<log4cxx::spi::LoggingEventPtr>& list = appender->getList();
  for (std::list<log4cxx::spi::LoggingEventPtr>::const_iterator it = list.begin(); it != list.end(); it++)
  {
    const log4cxx::LogString& msg = (*it)->getMessage();
    size_t pos_error = msg.find("Service call failed:");
    size_t pos_exception = msg.find(EXCEPTION);
    if (pos_error != std::string::npos && pos_exception != std::string::npos)
    {
      found_error_output = true;
    }
  }
  ASSERT_TRUE(found_error_output);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "service_exception");
  return RUN_ALL_TESTS();
}
