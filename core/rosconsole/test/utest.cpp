/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/console.h"

#include "log4cxx/appenderskeleton.h"
#include "log4cxx/spi/loggingevent.h"

#include <vector>
#include <stdexcept>

#include <gtest/gtest.h>

class TestAppender : public log4cxx::AppenderSkeleton
{
public:
  struct Info
  {
    log4cxx::LevelPtr level_;
    std::string message_;
    std::string logger_name_;
  };

  typedef std::vector<Info> V_Info;

  V_Info info_;

protected:
  virtual void append(const log4cxx::spi::LoggingEventPtr& event, log4cxx::helpers::Pool& pool)
  {
    Info info;
    info.level_ = event->getLevel();
    info.message_ = event->getMessage();
    info.logger_name_ = event->getLoggerName();

    info_.push_back( info );
  }

  virtual void close()
  {
  }
  virtual bool requiresLayout() const
  {
    return false;
  }
};

class TestAppenderWithThrow : public log4cxx::AppenderSkeleton
{
protected:
  virtual void append(const log4cxx::spi::LoggingEventPtr& event, log4cxx::helpers::Pool& pool)
  {
    throw std::runtime_error("This should be caught");
  }

  virtual void close()
  {
  }
  virtual bool requiresLayout() const
  {
    return false;
  }
};

#define DEFINE_COND_TESTS(name, macro_base, level) \
  TEST(RosConsole, name##Cond) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_COND(true, "Testing %d %d %d", 1, 2, 3); \
    macro_base##_COND(false, "Testing %d %d %d", 1, 2, 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##NamedCond) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_COND_NAMED(true, "test", "Testing %d %d %d", 1, 2, 3); \
    macro_base##_COND_NAMED(false, "test", "Testing %d %d %d", 1, 2, 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    EXPECT_STREQ(appender->info_[0].logger_name_.c_str(), ROSCONSOLE_ROOT_LOGGER_NAME".rosconsole.test"); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##StreamCond) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_STREAM_COND(true, "Testing " << 1 << " " << 2 << " " << 3); \
    macro_base##_STREAM_COND(false, "Testing " << 1 << " " << 2 << " " << 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##StreamCondNamed) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_STREAM_COND_NAMED(true, "test", "Testing " << 1 << " " << 2 << " " << 3); \
    macro_base##_STREAM_COND_NAMED(false, "test", "Testing " << 1 << " " << 2 << " " << 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    EXPECT_STREQ(appender->info_[0].logger_name_.c_str(), ROSCONSOLE_ROOT_LOGGER_NAME".rosconsole.test"); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  }

#define DEFINE_ONCE_TESTS(name, macro_base, level) \
  TEST(RosConsole, name##Once) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_ONCE("Testing %d %d %d", 1, 2, 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##NamedOnce) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_ONCE_NAMED("test", "Testing %d %d %d", 1, 2, 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    EXPECT_STREQ(appender->info_[0].logger_name_.c_str(), ROSCONSOLE_ROOT_LOGGER_NAME".rosconsole.test"); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##StreamOnce) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_STREAM_ONCE("Testing " << 1 << " " << 2 << " " << 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##StreamOnceNamed) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_STREAM_ONCE_NAMED("test", "Testing " << 1 << " " << 2 << " " << 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    EXPECT_STREQ(appender->info_[0].logger_name_.c_str(), ROSCONSOLE_ROOT_LOGGER_NAME".rosconsole.test"); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  }

#define DEFINE_LIMIT_TESTS(name, macro_base, level) \
  TEST(RosConsole, name##Limit) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_LIMIT(0.5, "Testing %d %d %d", 1, 2, 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##NamedLimit) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_LIMIT_NAMED(0.5, "test", "Testing %d %d %d", 1, 2, 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    EXPECT_STREQ(appender->info_[0].logger_name_.c_str(), ROSCONSOLE_ROOT_LOGGER_NAME".rosconsole.test"); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##StreamLimit) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_STREAM_LIMIT(0.5, "Testing " << 1 << " " << 2 << " " << 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##StreamLimitNamed) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_STREAM_LIMIT_NAMED(0.5, "test", "Testing " << 1 << " " << 2 << " " << 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    EXPECT_STREQ(appender->info_[0].logger_name_.c_str(), ROSCONSOLE_ROOT_LOGGER_NAME".rosconsole.test"); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  }

#define DEFINE_LEVEL_TESTS(name, macro_base, level) \
  TEST(RosConsole, name) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base("Testing %d %d %d", 1, 2, 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##Named) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_NAMED("test", "Testing %d %d %d", 1, 2, 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    EXPECT_STREQ(appender->info_[0].logger_name_.c_str(), ROSCONSOLE_ROOT_LOGGER_NAME".rosconsole.test"); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##Stream) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_STREAM("Testing " << 1 << " " << 2 << " " << 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  TEST(RosConsole, name##StreamNamed) \
  { \
    TestAppender* appender = new TestAppender; \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender( appender ); \
    macro_base##_STREAM_NAMED("test", "Testing " << 1 << " " << 2 << " " << 3); \
    ASSERT_EQ((int)appender->info_.size(), 1); \
    EXPECT_STREQ(appender->info_[0].message_.c_str(), "Testing 1 2 3"); \
    EXPECT_EQ(appender->info_[0].level_, ros::console::g_level_lookup[level]); \
    EXPECT_STREQ(appender->info_[0].logger_name_.c_str(), ROSCONSOLE_ROOT_LOGGER_NAME".rosconsole.test"); \
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->removeAppender( appender ); \
  } \
  DEFINE_COND_TESTS(name, macro_base, level) \
  DEFINE_ONCE_TESTS(name, macro_base, level) \
  DEFINE_LIMIT_TESTS(name, macro_base, level)

DEFINE_LEVEL_TESTS(debug, ROS_DEBUG, ros::console::levels::Debug)
DEFINE_LEVEL_TESTS(info, ROS_INFO, ros::console::levels::Info)
DEFINE_LEVEL_TESTS(warn, ROS_WARN, ros::console::levels::Warn)
DEFINE_LEVEL_TESTS(error, ROS_ERROR, ros::console::levels::Error)
DEFINE_LEVEL_TESTS(fatal, ROS_FATAL, ros::console::levels::Fatal)

TEST(RosConsole, loggingLevels)
{
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  TestAppender* appender = new TestAppender;
  logger->addAppender( appender );

  int pre_count = 0;
  int post_count = 0;

  {
    logger->setLevel( log4cxx::Level::getInfo() );
    pre_count = appender->info_.size();
    ROS_DEBUG("test");
    ROS_INFO("test");
    ROS_WARN("test");
    ROS_ERROR("test");
    ROS_FATAL("test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 4);

    logger->setLevel( log4cxx::Level::getWarn() );
    pre_count = appender->info_.size();
    ROS_DEBUG("test");
    ROS_INFO("test");
    ROS_WARN("test");
    ROS_ERROR("test");
    ROS_FATAL("test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 3);

    logger->setLevel( log4cxx::Level::getError() );
    pre_count = appender->info_.size();
    ROS_DEBUG("test");
    ROS_INFO("test");
    ROS_WARN("test");
    ROS_ERROR("test");
    ROS_FATAL("test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 2);

    logger->setLevel( log4cxx::Level::getFatal() );
    pre_count = appender->info_.size();
    ROS_DEBUG("test");
    ROS_INFO("test");
    ROS_WARN("test");
    ROS_ERROR("test");
    ROS_FATAL("test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 1);

    logger->setLevel( log4cxx::Level::getOff() );
    pre_count = appender->info_.size();
    ROS_DEBUG("test");
    ROS_INFO("test");
    ROS_WARN("test");
    ROS_ERROR("test");
    ROS_FATAL("test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count);
  }

  {
    logger->setLevel( log4cxx::Level::getInfo() );
    pre_count = appender->info_.size();
    ROS_DEBUG_STREAM("test");
    ROS_INFO_STREAM("test");
    ROS_WARN_STREAM("test");
    ROS_ERROR_STREAM("test");
    ROS_FATAL_STREAM("test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 4);

    logger->setLevel( log4cxx::Level::getWarn() );
    pre_count = appender->info_.size();
    ROS_DEBUG_STREAM("test");
    ROS_INFO_STREAM("test");
    ROS_WARN_STREAM("test");
    ROS_ERROR_STREAM("test");
    ROS_FATAL_STREAM("test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 3);

    logger->setLevel( log4cxx::Level::getError() );
    pre_count = appender->info_.size();
    ROS_DEBUG_STREAM("test");
    ROS_INFO_STREAM("test");
    ROS_WARN_STREAM("test");
    ROS_ERROR_STREAM("test");
    ROS_FATAL_STREAM("test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 2);

    logger->setLevel( log4cxx::Level::getFatal() );
    pre_count = appender->info_.size();
    ROS_DEBUG_STREAM("test");
    ROS_INFO_STREAM("test");
    ROS_WARN_STREAM("test");
    ROS_ERROR_STREAM("test");
    ROS_FATAL_STREAM("test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 1);

    logger->setLevel( log4cxx::Level::getOff() );
    pre_count = appender->info_.size();
    ROS_DEBUG_STREAM("test");
    ROS_INFO_STREAM("test");
    ROS_WARN_STREAM("test");
    ROS_ERROR_STREAM("test");
    ROS_FATAL_STREAM("test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count);
  }

  {
    logger->setLevel( log4cxx::Level::getInfo() );
    pre_count = appender->info_.size();
    ROS_DEBUG_NAMED("test_name", "test");
    ROS_INFO_NAMED("test_name", "test");
    ROS_WARN_NAMED("test_name", "test");
    ROS_ERROR_NAMED("test_name", "test");
    ROS_FATAL_NAMED("test_name", "test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 4);

    logger->setLevel( log4cxx::Level::getWarn() );
    pre_count = appender->info_.size();
    ROS_DEBUG_NAMED("test_name", "test");
    ROS_INFO_NAMED("test_name", "test");
    ROS_WARN_NAMED("test_name", "test");
    ROS_ERROR_NAMED("test_name", "test");
    ROS_FATAL_NAMED("test_name", "test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 3);

    logger->setLevel( log4cxx::Level::getError() );
    pre_count = appender->info_.size();
    ROS_DEBUG_NAMED("test_name", "test");
    ROS_INFO_NAMED("test_name", "test");
    ROS_WARN_NAMED("test_name", "test");
    ROS_ERROR_NAMED("test_name", "test");
    ROS_FATAL_NAMED("test_name", "test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 2);

    logger->setLevel( log4cxx::Level::getFatal() );
    pre_count = appender->info_.size();
    ROS_DEBUG_NAMED("test_name", "test");
    ROS_INFO_NAMED("test_name", "test");
    ROS_WARN_NAMED("test_name", "test");
    ROS_ERROR_NAMED("test_name", "test");
    ROS_FATAL_NAMED("test_name", "test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 1);

    logger->setLevel( log4cxx::Level::getOff() );
    pre_count = appender->info_.size();
    ROS_DEBUG_NAMED("test_name", "test");
    ROS_INFO_NAMED("test_name", "test");
    ROS_WARN_NAMED("test_name", "test");
    ROS_ERROR_NAMED("test_name", "test");
    ROS_FATAL_NAMED("test_name", "test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count);
  }

  {
    logger->setLevel( log4cxx::Level::getInfo() );
    pre_count = appender->info_.size();
    ROS_DEBUG_STREAM_NAMED("test_name", "test");
    ROS_INFO_STREAM_NAMED("test_name", "test");
    ROS_WARN_STREAM_NAMED("test_name", "test");
    ROS_ERROR_STREAM_NAMED("test_name", "test");
    ROS_FATAL_STREAM_NAMED("test_name", "test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 4);

    logger->setLevel( log4cxx::Level::getWarn() );
    pre_count = appender->info_.size();
    ROS_DEBUG_STREAM_NAMED("test_name", "test");
    ROS_INFO_STREAM_NAMED("test_name", "test");
    ROS_WARN_STREAM_NAMED("test_name", "test");
    ROS_ERROR_STREAM_NAMED("test_name", "test");
    ROS_FATAL_STREAM_NAMED("test_name", "test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 3);

    logger->setLevel( log4cxx::Level::getError() );
    pre_count = appender->info_.size();
    ROS_DEBUG_STREAM_NAMED("test_name", "test");
    ROS_INFO_STREAM_NAMED("test_name", "test");
    ROS_WARN_STREAM_NAMED("test_name", "test");
    ROS_ERROR_STREAM_NAMED("test_name", "test");
    ROS_FATAL_STREAM_NAMED("test_name", "test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 2);

    logger->setLevel( log4cxx::Level::getFatal() );
    pre_count = appender->info_.size();
    ROS_DEBUG_STREAM_NAMED("test_name", "test");
    ROS_INFO_STREAM_NAMED("test_name", "test");
    ROS_WARN_STREAM_NAMED("test_name", "test");
    ROS_ERROR_STREAM_NAMED("test_name", "test");
    ROS_FATAL_STREAM_NAMED("test_name", "test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count + 1);

    logger->setLevel( log4cxx::Level::getOff() );
    pre_count = appender->info_.size();
    ROS_DEBUG_STREAM_NAMED("test_name", "test");
    ROS_INFO_STREAM_NAMED("test_name", "test");
    ROS_WARN_STREAM_NAMED("test_name", "test");
    ROS_ERROR_STREAM_NAMED("test_name", "test");
    ROS_FATAL_STREAM_NAMED("test_name", "test");
    post_count = appender->info_.size();
    EXPECT_EQ(post_count, pre_count);
  }

  logger->removeAppender( appender );
}

TEST(RosConsole, changingLevel)
{
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  TestAppender* appender = new TestAppender;
  logger->addAppender( appender );

  logger->setLevel( log4cxx::Level::getError() );
  for ( int i = ros::console::levels::Debug; i < ros::console::levels::Count; ++i )
  {
    ROS_LOG((ros::console::Level)i, ROSCONSOLE_DEFAULT_NAME, "test");
  }

  EXPECT_EQ((int)appender->info_.size(), 2);

  logger->removeAppender( appender );

  logger->setLevel( log4cxx::Level::getDebug() );
}

TEST(RosConsole, changingLoggerLevel)
{
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  TestAppender* appender = new TestAppender;
  logger->addAppender( appender );

  for ( int i = ros::console::levels::Fatal; i >= 0; --i )
  {
    logger->setLevel( ros::console::g_level_lookup[i] );
    ros::console::notifyLoggerLevelsChanged();
    ROS_LOG((ros::console::Level)i, ROSCONSOLE_DEFAULT_NAME, "test");
  }

  EXPECT_EQ((int)appender->info_.size(), 5);

  logger->removeAppender( appender );

  logger->setLevel( log4cxx::Level::getDebug() );
}

TEST(RosConsole, longPrintfStyleOutput)
{
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  TestAppender* appender = new TestAppender;
  logger->addAppender( appender );

  std::stringstream ss;
  for (int i = 0; i < 100000; ++i )
  {
    ss << 'a';
  }

  ROS_INFO("%s", ss.str().c_str());

  ASSERT_EQ((int)appender->info_.size(), 1);
  EXPECT_STREQ(appender->info_[0].message_.c_str(), ss.str().c_str());

  logger->removeAppender( appender );

  logger->setLevel( log4cxx::Level::getDebug() );
}

TEST(RosConsole, throwingAppender)
{
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  TestAppenderWithThrow* appender = new TestAppenderWithThrow;
  logger->addAppender( appender );

  try
  {
      ROS_INFO("Hello there");
  }
  catch (std::exception& e)
  {
      FAIL();
  }

  logger->removeAppender( appender );
  logger->setLevel( log4cxx::Level::getDebug() );
}

void onceFunc()
{
  ROS_LOG_ONCE(ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, "Hello");
}

TEST(RosConsole, once)
{
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  TestAppender* appender = new TestAppender;
  logger->addAppender(appender);

  onceFunc();
  onceFunc();

  EXPECT_EQ(appender->info_.size(), 1ULL);

  logger->removeAppender(appender);
}

void limitFunc()
{
  ROS_LOG_LIMIT(2.0, ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, "Hello");
}

TEST(RosConsole, limit)
{
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  TestAppender* appender = new TestAppender;
  logger->addAppender(appender);

  ros::Time start = ros::Time::now();
  while (ros::Time::now() <= start + ros::Duration(2.0))
  {
    limitFunc();
    ros::Duration(0.01).sleep();
  }

  limitFunc();

  EXPECT_EQ(appender->info_.size(), 2ULL);

  logger->removeAppender(appender);
}

void onceStreamFunc()
{
  ROS_LOG_STREAM_ONCE(ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, "Hello");
}

TEST(RosConsole, onceStream)
{
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  TestAppender* appender = new TestAppender;
  logger->addAppender(appender);

  onceStreamFunc();
  onceStreamFunc();

  EXPECT_EQ(appender->info_.size(), 1ULL);

  logger->removeAppender(appender);
}

void limitStreamFunc()
{
  ROS_LOG_STREAM_LIMIT(2.0, ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, "Hello");
}

TEST(RosConsole, limitStream)
{
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  TestAppender* appender = new TestAppender;
  logger->addAppender(appender);

  ros::Time start = ros::Time::now();
  while (ros::Time::now() <= start + ros::Duration(2.0))
  {
    limitStreamFunc();
    ros::Duration(0.01).sleep();
  }

  limitStreamFunc();

  EXPECT_EQ(appender->info_.size(), 2ULL);

  logger->removeAppender(appender);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ROSCONSOLE_AUTOINIT;
  log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME)->removeAllAppenders();
  log4cxx::Logger::getRootLogger()->setLevel(log4cxx::Level::getDebug());
  log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME)->setLevel(log4cxx::Level::getDebug());

  return RUN_ALL_TESTS();
}
