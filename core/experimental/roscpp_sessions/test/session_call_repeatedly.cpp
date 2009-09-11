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

using namespace std;
using namespace ros;

class SimpleSessionClient
{
public:
    SimpleSessionClient()
    {
        roscpp_sessions::simple_session::Request req;
        roscpp_sessions::simple_session::Response res;
        req.options = 1234;
        handle = session::create_session("session_adv",req,res);

        if( !!handle ) {
            printf("session %s:%d open\n", handle->GetSessionName().c_str(), handle->GetSessionId());
            test();

            printf("closing all connections and resuming session\n");
            session::abstractSessionHandle pnewhandle(new session::Session<roscpp_sessions::simple_session::Request, roscpp_sessions::simple_session::Response>(handle->GetSessionName(), handle->GetSessionId()));
            handle = pnewhandle;
            test();

            handle->terminate();
        }
    }

    void test() {
        int N = 100;
        for(int i = 0; i < N; ++i) {
            int a = rand(), b = rand(), c = a+b;
            setreq.variable = "a";
            setreq.value = a;
            if( !handle->call("set_variable",setreq,setres) ) {
                printf("failed to call set_variable");
                return;
            }

            setreq.variable = "b";
            setreq.value = b;
            if( !handle->call("set_variable",setreq,setres) ) {
                printf("failed to call set_variable");
                return;
            }

            getreq.variable = "a";
            if( !handle->call("get_variable",getreq,getres) ) {
                printf("failed to call get_variable");
                return;
            }
            ROS_ASSERT(getres.result==a);

            addreq.variable1 = "a";
            addreq.variable2 = "b";
            addreq.result = "c";
            if( !handle->call("add_variables",addreq,addres) ) {
                printf("failed to call add_variables");
                return;
            }

            getreq.variable = "c";
            if( !handle->call("get_variable",getreq,getres) ) {
                printf("failed to call get_variable");
                return;
            }

            printf("%d + %d = %d(res=%d)", a, b, c, getres.result);
            ROS_ASSERT(getres.result==c);
        }
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

int
main(int argc, char** argv)
{
    ros::init(argc, argv);
    ros::Node n("caller");

    ros::service::waitForService("mysession");

    struct timespec sleep_time = {0, 2000000};
    while(n.ok()) {
        SimpleSessionClient client;
        nanosleep(&sleep_time,NULL);
    }


    return 0;
}
