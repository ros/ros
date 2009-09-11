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
#include <boost/shared_ptr.hpp>

#include "roscpp_sessions/simple_session.h"
#include "roscpp_sessions/set_variable.h"
#include "roscpp_sessions/get_variable.h"
#include "roscpp_sessions/add_variables.h"

#include <map>

using namespace std;
using namespace ros;

// keeps track of created variables
class SimpleSessionInstance
{
public:
    void set_variable(const string& name, int val) { variables[name] = val; }
    int get_variable(const string& name) { return variables[name]; }
    void add_variables(const string& result, const string& name1, const string& name2) {
        variables[result] = variables[name1] + variables[name2];
    }

private:
    map<string,int> variables;
};

class SimpleSession
{
public:
    SimpleSession()
    {
    }

    void advertise_sessions() {
        fprintf(stderr,"starting to advertise\n");
        Node::instance()->advertiseService("session_adv",&SimpleSession::startsession,this,1);

        // advertise persistent services, the protocol for these differs!
        Node::instance()->advertiseService("set_variable",&SimpleSession::set_variable,this,-1);
        Node::instance()->advertiseService("get_variable",&SimpleSession::get_variable,this,-1);
        Node::instance()->advertiseService("add_variables",&SimpleSession::add_variables,this,-1);
        fprintf(stderr,"end advertise\n");
    }

    template <class MReq>
    SimpleSessionInstance* getstate(const MReq& req)
    {
        if( !req.__connection_header )
            return NULL;

        boost::mutex::scoped_lock lock(map_mutex);
        ros::M_string::const_iterator it = req.__connection_header->find("session_adv");
        if( it == req.__connection_header->end() )
            return NULL;

        int sessionid = atoi(it->second.c_str());
        if( mapsessions.find(sessionid) == mapsessions.end() )
            return NULL;
        return mapsessions[sessionid].get();
    }

    void unadvertise_sessions() {
        fprintf(stderr,"starting to unadvertise add_variables\n");
        Node::instance()->unadvertiseService("add_variables");
        fprintf(stderr,"starting to unadvertise get_variables\n");
        Node::instance()->unadvertiseService("get_variable");
        fprintf(stderr,"starting to unadvertise set_variables\n");
        Node::instance()->unadvertiseService("set_variable");
        fprintf(stderr,"starting to unadvertise session_adv\n");
        Node::instance()->unadvertiseService("session_adv");
        fprintf(stderr,"end unadvertise\n");
    }

    bool startsession(roscpp_sessions::simple_session::Request& req, roscpp_sessions::simple_session::Response& res) {
        fprintf(stderr,"start session\n");
        boost::mutex::scoped_lock lock(map_mutex);
        
        if( req.sessionid ) {
            // destroy
            cout << "terminate session: " << req.sessionid << endl;
            mapsessions.erase(req.sessionid);
        }
        else {
            // start a new session with id
            int id = rand();
            ROS_ASSERT( mapsessions.find(id) == mapsessions.end() );
            mapsessions[id].reset(new SimpleSessionInstance());
            cout << "simple session " << id << " started with options " << req.options << endl;
            res.sessionid = id;
        }

        fprintf(stderr,"end session\n");
        return true;
    }

    bool set_variable(roscpp_sessions::set_variable::Request& req,
                      roscpp_sessions::set_variable::Response& res)
    {
        SimpleSessionInstance* pinst = getstate(req);
        if(pinst == NULL)
            return false;
        pinst->set_variable(req.variable,req.value);
        return true;
    }

    bool get_variable(roscpp_sessions::get_variable::Request& req,
                      roscpp_sessions::get_variable::Response& res)
    {
        SimpleSessionInstance* pinst = getstate(req);
        if(pinst == NULL)
            return false;
        res.result = pinst->get_variable(req.variable);
        return true;
    }

    bool add_variables(roscpp_sessions::add_variables::Request& req,
                      roscpp_sessions::add_variables::Response& res)
    {
        SimpleSessionInstance* pinst = getstate(req);
        if(pinst == NULL)
            return false;
        pinst->add_variables(req.result, req.variable1,req.variable2);
        return true;
    }

private:
    map<int,boost::shared_ptr<SimpleSessionInstance> > mapsessions;
    boost::mutex map_mutex;
};

TEST(SessionAdv, Simple)
{
    SimpleSession ss;
    Node::instance()->advertiseService("session_adv",&SimpleSession::startsession,&ss);
    usleep(400000);
    Node::instance()->unadvertiseService("session_adv");
    fprintf(stderr,"EndSimple\n");
}

TEST(SessionAdv, Normal)
{
    for(int i = 0; i < 5; ++i) {
        SimpleSession ss;
        ss.advertise_sessions();
        usleep(200000);
        ss.unadvertise_sessions();
    }
    fprintf(stderr,"EndNormal\n");
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv);
    ros::Node n("session");

    int ret = RUN_ALL_TESTS();
    fprintf(stderr,"After RUN_ALL_TESTS\n");

    return ret;
}
