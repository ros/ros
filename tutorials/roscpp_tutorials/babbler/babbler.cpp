///////////////////////////////////////////////////////////////////////////////
// The roscpp_demo package has a few demos of the roscpp c++ client library
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include "ros/node.h"
#include "std_msgs/String.h"

#include <sstream>

void speak(int i)
{
  /**
   * This is the message object. You stuff it with data, and then publish it.
   */
  std_msgs::String msg;

  std::stringstream ss;
  ss << "hello world " << i;
  ROS_INFO("%s", ss.str().c_str());
  msg.data = ss.str();

  /**
   * The publish() function is how you send messages. The first parameter
   * is a topic name, and there must have been an advertise<>() call on this
   * topic name first. (If you try to publish on a non-advertised topic name,
   * the client library will print an error and exit.) The second parameter
   * is the message object. The type of this object must agree with the type
   * given as a template parameter to the advertise<>() call, as was done
   * in the constructor above.
   */
  ros::Node::instance()->publish("babble", msg);
}

/**
 * ROS does not wrap your main. This means that you have to call the ROS
 * startup (init) and shutdown (fini) functions manually. We view this as a
 * feature -- you maintain full control over the execution environment
 * of your program and how it interacts with ROS. In a simple program like this
 * one, it may not be a big deal, but when large GUI libraries get involved,
 * it is much nicer to be able to call ROS::init  and ROS:fini at convenient
 * times.
 *
 * The ros::init() function needs to see argc and argv so that it can perform
 * any namespace mangling that was provided at the command line. You can get
 * around this if you want by constructing your own array of variable mappings
 * and passing that to ROS::init(), but for most command-line programs,
 * passing argc and argv is the easiest way to do it.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("babbler");

  /**
   * The advertise<> function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.
   */
  n.advertise<std_msgs::String>("babble", 1000);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (n.ok())
  {
    usleep(100000);

    speak(count);
    ++count;
  }

  
  return 0;
}

