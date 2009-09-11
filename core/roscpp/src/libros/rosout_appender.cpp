/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/rosout_appender.h"
#include "ros/node.h"
#include <roslib/Log.h>

#include <log4cxx/spi/loggingevent.h>

namespace ros
{

ROSOutAppender::ROSOutAppender(ros::Node* node)
: node_(node)
{
  node_->advertise<roslib::Log>("/rosout", 0);

  if (node_->ok())
  {
    topic_ = PublicationWPtr(node_->getTopic("/rosout"));
    ROS_ASSERT(topic_.lock());
  }
}

ROSOutAppender::~ROSOutAppender()
{

}

const std::string&  ROSOutAppender::getLastError()
{
  return last_error_;
}

void ROSOutAppender::append(const log4cxx::spi::LoggingEventPtr& event, log4cxx::helpers::Pool& pool)
{
  // If the node is in the middle of shutting down, don't do anything
  if (node_->shuttingDown())
  {
    return;
  }

  if (PublicationPtr topic = topic_.lock())
  {
    roslib::Log msg;

    if (event->getLevel() == log4cxx::Level::getFatal())
    {
      msg.level = roslib::Log::FATAL;
      last_error_ = event->getMessage();
    }
    else if (event->getLevel() == log4cxx::Level::getError())
    {
      msg.level = roslib::Log::ERROR;
      last_error_ = event->getMessage();
    }
    else if (event->getLevel() == log4cxx::Level::getWarn())
    {
      msg.level = roslib::Log::WARN;
    }
    else if (event->getLevel() == log4cxx::Level::getInfo())
    {
      msg.level = roslib::Log::INFO;
    }
    else if (event->getLevel() == log4cxx::Level::getDebug())
    {
      msg.level = roslib::Log::DEBUG;
    }

    msg.name = node_->getName();
    msg.msg = event->getMessage();

    const log4cxx::spi::LocationInfo& info = event->getLocationInformation();
    msg.file = info.getFileName();
    msg.function = info.getMethodName();
    msg.line = info.getLineNumber();

    /// \todo Get the topic list somehow
    node_->getAdvertisedTopics(msg.topics);

    //node_->publish("/rosout", msg);
    node_->publish(topic, msg);
  }
}

} // namespace ros
