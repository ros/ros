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
#include <gtest/gtest.h>

#include <ros/session.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <ros/message.h>
#include "roscpp_sessions/simple_session.h"
#include "roscpp_sessions/set_variable.h"
#include "roscpp_sessions/get_variable.h"
#include "roscpp_sessions/add_variables.h"

#include <map>
#include <list>

using namespace std;
using namespace ros;

TEST(SessionCall, CreateTerminate)
{
    roscpp_sessions::simple_session::Request req;
    roscpp_sessions::simple_session::Response res;
    req.options = 1234;
    session::abstractSessionHandle handle = session::create_session("session_adv",req,res);
    ASSERT_TRUE(handle != NULL);
    ASSERT_TRUE(handle->GetSessionId() != 0 );

    fprintf(stderr,"session %s:%d open\n", handle->GetSessionName().c_str(), handle->GetSessionId());

    fprintf(stderr, "terminating first\n");
    handle->terminate();
    handle.reset();

    handle = session::create_session("session_adv",req,res);
    ASSERT_TRUE(!!handle);
    ASSERT_TRUE(handle->GetSessionId() != 0 );

    fprintf(stderr, "session %s:%d open\n", handle->GetSessionName().c_str(), handle->GetSessionId());

    fprintf(stderr, "deleting\n");
    handle.reset();
}

TEST(SessionCall, CreateTerminateComplex)
{
    const int N = 20;
    session::abstractSessionHandle handles[N];
    roscpp_sessions::simple_session::Request req;
    roscpp_sessions::simple_session::Response res;
    roscpp_sessions::set_variable::Request setreq;
    roscpp_sessions::set_variable::Response setres;
    roscpp_sessions::get_variable::Request getreq;
    roscpp_sessions::get_variable::Response getres;

    for(int i = 0; i < N; ++i) {
        req.options = i;
        handles[i] = session::create_session("session_adv",req,res);
        ASSERT_TRUE(!!handles[i]);
        ASSERT_TRUE(handles[i]->GetSessionId() != 0 );
    }

    for(int iter = 0; iter < 10; ++iter ) {
        for(int i = 0; i < N; ++i) {
            setreq.variable = "asdf";
            setreq.value = i;
            handles[i]->call("set_variable",setreq,setres);
        }

        for(int i = 0; i < N; ++i) {
            getreq.variable = "asdf";
            handles[i]->call("get_variable",getreq,getres);
            EXPECT_EQ(i, getres.result);
        }

        // do again but this time asynchronously
        for(int i = 0; i < N; ++i) {
            setreq.variable = "asdf";
            setreq.value = i*i;
            handles[i]->call("set_variable",setreq,setres,true);
        }

        for(int i = 0; i < N; ++i) {
            getreq.variable = "asdf";
            handles[i]->call("get_variable",getreq,getres);
            EXPECT_EQ(i*i, getres.result);
            ROS_ASSERT(i*i == getres.result);
        }
    }

    for(int i = 1; i < N; ++i)
        handles[i].reset();

    setreq.variable = "hmmmm";
    setreq.value = 1234;
    handles[0]->call("set_variable",setreq,setres,true);
    getreq.variable = "hmmmm";
    handles[0]->call("get_variable",getreq,getres);
    EXPECT_EQ(1234, getres.result);
    handles[0].reset();
}

TEST(SessionCall, CreateSyncCommands)
{
    roscpp_sessions::simple_session::Request req;
    roscpp_sessions::simple_session::Response res;
    req.options = 1234;
    session::abstractSessionHandle handle = session::create_session("session_adv",req,res);
    ASSERT_TRUE(!!handle);

    roscpp_sessions::set_variable::Request setreq;
    roscpp_sessions::set_variable::Response setres;
    roscpp_sessions::get_variable::Request getreq;
    roscpp_sessions::get_variable::Response getres;
    roscpp_sessions::add_variables::Request addreq;
    roscpp_sessions::add_variables::Response addres;

    int N = 100;
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
        EXPECT_EQ(a,getres.result);

        addreq.variable1 = "a";
        addreq.variable2 = "b";
        addreq.result = "c";
        handle->call("add_variables",addreq,addres);

        getreq.variable = "c";
        handle->call("get_variable",getreq,getres);
        EXPECT_EQ(c,getres.result);
    }
}

TEST(SessionCall, CreateAsyncCommands)
{
    roscpp_sessions::simple_session::Request req;
    roscpp_sessions::simple_session::Response res;
    req.options = 1234;
    session::abstractSessionHandle handle = session::create_session("session_adv",req,res);
    ASSERT_TRUE(!!handle);

    roscpp_sessions::set_variable::Request setreq;
    roscpp_sessions::set_variable::Response setres;
    roscpp_sessions::get_variable::Request getreq;
    roscpp_sessions::get_variable::Response getres;
    roscpp_sessions::add_variables::Request addreq;
    roscpp_sessions::add_variables::Response addres;

    // should complete relatively fast
    int N = 2000;
    for(int i = 0; i < N; ++i) {
        int a = rand(), b = rand();
        setreq.variable = "a";
        setreq.value = a;
        handle->call("set_variable",setreq,setres,true);

        setreq.variable = "b";
        setreq.value = b;
        handle->call("set_variable",setreq,setres,true);
    }

    N = 100;
    for(int i = 0; i < N; ++i) {
        int a = rand(), b = rand(), c = a+b;
        setreq.variable = "a";
        setreq.value = a;
        handle->call("set_variable",setreq,setres,true);

        setreq.variable = "b";
        setreq.value = b;
        handle->call("set_variable",setreq,setres,true);

        getreq.variable = "a";
        handle->call("get_variable",getreq,getres);
        EXPECT_EQ(a,getres.result);

        addreq.variable1 = "a";
        addreq.variable2 = "b";
        addreq.result = "c";
        handle->call("add_variables",addreq,addres,true);

        getreq.variable = "c";
        handle->call("get_variable",getreq,getres);
        EXPECT_EQ(c,getres.result);
    }

    handle->terminate();
    handle->terminate();
    handle->terminate();
}

TEST(SessionCall, SyncCommandsTerm)
{
    roscpp_sessions::simple_session::Request req;
    roscpp_sessions::simple_session::Response res;
    req.options = 1234;
    session::abstractSessionHandle handle = session::create_session("session_adv",req,res);
    ASSERT_TRUE(!!handle);

    roscpp_sessions::set_variable::Request setreq;
    roscpp_sessions::set_variable::Response setres;
    roscpp_sessions::get_variable::Request getreq;
    roscpp_sessions::get_variable::Response getres;
    roscpp_sessions::add_variables::Request addreq;
    roscpp_sessions::add_variables::Response addres;

    int N = 100;
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
        ASSERT_TRUE(getres.result==a);

        addreq.variable1 = "a";
        addreq.variable2 = "b";
        addreq.result = "c";
        handle->call("add_variables",addreq,addres);

        getreq.variable = "c";
        handle->call("get_variable",getreq,getres);
        ASSERT_TRUE(getres.result==c);
    }

    session::abstractSessionHandle pnewhandle(new session::Session<roscpp_sessions::simple_session::Request, roscpp_sessions::simple_session::Response>(handle->GetSessionName(), handle->GetSessionId()));
    handle = pnewhandle;

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
        ASSERT_TRUE(getres.result==a);

        addreq.variable1 = "a";
        addreq.variable2 = "b";
        addreq.result = "c";
        handle->call("add_variables",addreq,addres);

        getreq.variable = "c";
        handle->call("get_variable",getreq,getres);
        ASSERT_TRUE(getres.result==c);
    }

    ASSERT_TRUE(handle->terminate());
}

void testsynccommandsterm(bool& bsuccess)
{
    bsuccess = false;
    roscpp_sessions::simple_session::Request req;
    roscpp_sessions::simple_session::Response res;
    req.options = 1234;
    session::abstractSessionHandle handle = session::create_session("session_adv",req,res);
    if( handle == NULL ) {
        fprintf(stderr, "failed to create handle\n");
        return;
    }

    roscpp_sessions::set_variable::Request setreq;
    roscpp_sessions::set_variable::Response setres;
    roscpp_sessions::get_variable::Request getreq;
    roscpp_sessions::get_variable::Response getres;
    roscpp_sessions::add_variables::Request addreq;
    roscpp_sessions::add_variables::Response addres;

    int N = 100;
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
        if(getres.result!=a) {
            fprintf(stderr, "a not correct\n");
            return;
        }

        addreq.variable1 = "a";
        addreq.variable2 = "b";
        addreq.result = "c";
        handle->call("add_variables",addreq,addres);

        getreq.variable = "c";
        handle->call("get_variable",getreq,getres);
        if(getres.result!=c) {
            fprintf(stderr, "c not correct\n");
            return;
        }
    }

    if( !handle->terminate() )
        return;

    bsuccess = true;
}

TEST(SessionCall, MultiSessions)
{
    roscpp_sessions::simple_session::Request req;
    roscpp_sessions::simple_session::Response res;
    req.options = 1234;
    session::abstractSessionHandle handle = session::create_session("session_adv",req,res);
    ASSERT_TRUE(!!handle);
    handle.reset();

    // create multiple threads of sessions
    list<boost::thread*> listthreads;
    list<bool> listsuccess;
    for(int i = 0; i < 20; ++i) {
        listsuccess.push_back(false);
        listthreads.push_back(new boost::thread(boost::bind(testsynccommandsterm,boost::ref(listsuccess.back()))));
    }

    // wait on all threads
    list<bool>::iterator itsuccess = listsuccess.begin();
    for(list<boost::thread*>::iterator it = listthreads.begin(); it != listthreads.end(); ++it, ++itsuccess) {
        (*it)->join();
        ASSERT_TRUE(*itsuccess);
        delete *it;
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv);
    ros::Node n("caller");

    ros::service::waitForService("session_adv");
    int ret = RUN_ALL_TESTS();

    

    return ret;
}
