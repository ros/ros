/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"

#include <boost/thread.hpp>

/**
 * This tutorial demonstrates the use of custom separate callback queues that can be processed
 * independently, whether in different threads or just at different times.
 */

/**
 * This callback gets called from the main queue processed in spin()
 */
void chatterCallbackMainQueue(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("I heard: [ " << msg->data << "] in thread [" << boost::this_thread::get_id() << "] (Main thread)");
}

/**
 * This callback gets called from the custom queue
 */
void chatterCallbackCustomQueue(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("I heard: [ " << msg->data << "] in thread [" << boost::this_thread::get_id() << "]");
}

/**
 * The custom queue used for one of the subscription callbacks
 */
ros::CallbackQueue g_queue;
void callbackThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  ros::NodeHandle n;
  while (n.ok())
  {
    g_queue.callAvailable(ros::WallDuration(0.01));
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_with_custom_callback_processing");
  ros::NodeHandle n;

  /**
   * The SubscribeOptions structure lets you specify a custom queue to use for a specific subscription.
   * You can also set a default queue on a NodeHandle using the NodeHandle::setCallbackQueue() function.
   *
   * AdvertiseOptions and AdvertiseServiceOptions offer similar functionality.
   */
  ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::String>("chatter", 1000,
                                                                              chatterCallbackCustomQueue,
                                                                              ros::VoidPtr(), &g_queue);
  ros::Subscriber sub = n.subscribe(ops);

  /**
   * Now we subscribe using the normal method, to demonstrate the difference.
   */
  ros::Subscriber sub2 = n.subscribe("chatter", 1000, chatterCallbackMainQueue);

  /**
   * Start a thread to service the custom queue
   */
  boost::thread chatter_thread(callbackThread);

  ROS_INFO_STREAM("Main thread id=" << boost::this_thread::get_id());
  /**
   * Now do a custom spin, to demonstrate the difference.
   */
  ros::Rate r(1);
  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  chatter_thread.join();

  return 0;
}
