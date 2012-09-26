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

/* Author: Brian Gerkey */

/*
 * Advertise a service
 */

#include "ros/ros.h"
#include <roscpp/TestStringString.h>

bool caseFlip(roscpp::TestStringString::Request  &req,
                     roscpp::TestStringString::Response &res)
{
  // copy over the request and overwrite the letters with their case-flip
  res.str = req.str;
  for (size_t i = 0; i < res.str.length(); i++)
  {
    char c = res.str[i];
    if (islower(c))
      c = toupper(c);
    else if (isupper(c))
      c = tolower(c);
    res.str[i] = c;
  }
  return true;
}

bool caseFlipLongRunning(roscpp::TestStringString::Request  &req,
                     roscpp::TestStringString::Response &res)
{
  caseFlip(req, res);

  ros::Duration(2).sleep();
  return true;
}

bool caseFlipUnadvertise(roscpp::TestStringString::Request  &req,
                     roscpp::TestStringString::Response &res, ros::ServiceServer& srv)
{
  caseFlip(req, res);

  srv.shutdown();

  ros::Duration(2).sleep();
  return true;
}


int
main(int argc, char** argv)
{
  ros::init(argc, argv, "service_adv");
  ros::NodeHandle nh;

  ros::ServiceServer srv1, srv2, srv3;
  srv1 = nh.advertiseService("service_adv", caseFlip);
  srv2 = nh.advertiseService("service_adv_long", caseFlipLongRunning);
  srv3 = nh.advertiseService<roscpp::TestStringString::Request, roscpp::TestStringString::Response>("service_adv_unadv_in_callback", boost::bind(caseFlipUnadvertise, _1, _2, boost::ref(srv3)));
  ros::spin();
}

