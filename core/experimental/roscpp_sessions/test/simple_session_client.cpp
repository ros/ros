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

#include "roscpp_sessions/simple_session.h"
#include "roscpp_sessions/set_variable.h"
#include "roscpp_sessions/get_variable.h"
#include "roscpp_sessions/add_variables.h"
#include <map>

using namespace std;
using namespace ros;

class SimpleSessionClient : public ros::Node
{
public:
    SimpleSessionClient() : ros::Node("simple_session_client")
    {
        roscpp_sessions::simple_session::Request req;
        roscpp_sessions::simple_session::Response res;
        req.options = 1234;
        handle = session::create_session("session_adv",req,res);

        printf("session %s:%d open\n", handle->GetSessionName().c_str(), handle->GetSessionId());
        testsync();
        testasync();

        printf("closing all connections and resuming session\n");
        session::abstractSessionHandle newhandle(new session::Session<roscpp_sessions::simple_session::Request, roscpp_sessions::simple_session::Response>(handle->GetSessionName(), handle->GetSessionId()));
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

        printf("made %d async calls in %fs\n", 2*N,(float)(Time::now()-start_time).toSec());

        // reset
        getreq.variable = "a";
        handle->call("get_variable",getreq,getres);
        printf("flushed all async calls\n");
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

        printf("made %d sync calls in %fs\n", 5*N,(float)(Time::now()-start_time).toSec());
    }

private:
    session::abstractSessionHandle handle;
    roscpp_sessions::set_variable::Request setreq;
    roscpp_sessions::set_variable::Response setres;
    roscpp_sessions::get_variable::Request getreq;
    roscpp_sessions::get_variable::Response getres;
    roscpp_sessions::add_variables::Request addreq;
    roscpp_sessions::add_variables::Response addres;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  SimpleSessionClient client;
  
  return 0;
}
