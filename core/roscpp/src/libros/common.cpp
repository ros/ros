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

#include "ros/common.h"
#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <cassert>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

using std::string;

std::string ros::getPackagePath(const std::string &package_name)
{
  // uses rospack to get a package path
  string cmd = string("rospack find ") + package_name;
  FILE *rospipe = popen(cmd.c_str(), "r");
  if (!rospipe)
    return string();
  char rosout[PATH_MAX];
  if (fgets(rosout, PATH_MAX, rospipe))
  {
    string ret(rosout);
    // scrape any newlines out of it
    for (size_t newline = ret.find('\n'); newline != string::npos;
                newline = ret.find('\n'))
      ret.erase(newline, 1);
    return ret;
  }
  return string(); // couldn't read from rospack pipe
}

void ros::disableAllSignalsInThisThread()
{
  sigset_t signal_set;

  /* block all signals */
  sigfillset( &signal_set );
  pthread_sigmask( SIG_BLOCK, &signal_set, NULL );
}
