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

// author: Rosen Diankov
#include <ros/session.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include "session_tutorials/simple_session.h"
#include "session_tutorials/set_variable.h"
#include "session_tutorials/get_variable.h"
#include "session_tutorials/add_variables.h"
#include <map>

using namespace std;
using namespace ros;

boost::shared_ptr<ros::Node> s_pmasternode;

class SimpleSessionClient
{
public:
    SimpleSessionClient()
    {
        session_tutorials::simple_session::Request req;
        session_tutorials::simple_session::Response res;
        req.options = 1234;
        handle = session::create_session("simple_session",req,res);

        ROS_INFO("session %s:%d open", handle->GetSessionName().c_str(), handle->GetSessionId());
        testsync();
        testasync();

        ROS_INFO("closing all connections and resuming session");
        session::abstractSessionHandle newhandle(new session::Session<session_tutorials::simple_session::Request, session_tutorials::simple_session::Response>(handle->GetSessionName(), handle->GetSessionId()));
        handle = newhandle;
        testsync();

        handle->terminate();
        handle.reset();
    }

    void testasync() {
        Time start_time = Time::now();
        int N = 10000;
        for(int i = 0; i < N; ++i) {
            int a = rand(), b = rand();
            setreq.variable = "a";
            setreq.value = a;
            handle->call("set_variable",setreq,setres,true);

            setreq.variable = "b";
            setreq.value = b;
            handle->call("set_variable",setreq,setres,true);
        }

        ROS_INFO("made %d async calls in %fs", 2*N,(float)(Time::now()-start_time).toSec());

        // reset
        getreq.variable = "a";
        handle->call("get_variable",getreq,getres);
        ROS_INFO("flushed all async calls");
    }

    void testsync() {
        Time start_time = Time::now();
        int N = 5000;
        for(int i = 0; i < N; ++i) {
            int a = rand(), b = rand(), c = a+b;
            setreq.variable = "a";
            setreq.value = a;
            handle->call("set_variable",setreq,setres);

            setreq.variable = "b";
            setreq.value = b;
            handle->call("set_variable",setreq,setres);

            getreq.variable = "a";
            handle->call("get_variable",getreq,getres);
            ROS_ASSERT(getres.result==a);
            //printf("a=%d\n", getres.result);

            addreq.variable1 = "a";
            addreq.variable2 = "b";
            addreq.result = "c";
            handle->call("add_variables",addreq,addres);

            getreq.variable = "c";
            handle->call("get_variable",getreq,getres);
            //ROS_INFO("%d + %d = %d(res=%d)", a, b, c, getres.result);
            ROS_ASSERT(getres.result==c);
            //printf("c=%d\n", getres.result);
        }

        ROS_INFO("made %d sync calls in %fs", 5*N,(float)(Time::now()-start_time).toSec());
    }

private:
    session::abstractSessionHandle handle;
    session_tutorials::set_variable::Request setreq;
    session_tutorials::set_variable::Response setres;
    session_tutorials::get_variable::Request getreq;
    session_tutorials::get_variable::Response getres;
    session_tutorials::add_variables::Request addreq;
    session_tutorials::add_variables::Response addres;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  s_pmasternode.reset(new ros::Node("simple_session_client"));
  if( !s_pmasternode->checkMaster() )
      return -1;

  boost::shared_ptr<SimpleSessionClient> client(new SimpleSessionClient());
  s_pmasternode->spin();
  client.reset();
  s_pmasternode.reset();
  return 0;
}
