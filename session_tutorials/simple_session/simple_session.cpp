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
#include <ros/node_handle.h>
#include <ros/session.h>
#include <ros/master.h>
#include <boost/thread/mutex.hpp>

#include "roscpp_sessions/simple_session.h"
#include "roscpp_sessions/set_variable.h"
#include "roscpp_sessions/get_variable.h"
#include "roscpp_sessions/add_variables.h"

#include <map>

using namespace std;
using namespace ros;

boost::shared_ptr<ros::NodeHandle> s_pmasternode;

// keeps track of created variables
class SimpleSessionInstance
{
public:
    void set_variable(const string& name, int val) { variables[name] = val; }
    int get_variable(const string& name) {
        ROS_ASSERT(variables.find(name)!=variables.end());
        return variables[name];
    }
    void add_variables(const string& result, const string& name1, const string& name2) {
        ROS_ASSERT(variables.find(name1)!=variables.end());
        ROS_ASSERT(variables.find(name2)!=variables.end());
        variables[result] = variables[name1] + variables[name2];
    }

private:
    map<string,int> variables;
};

class SimpleSession
{
    string _sessionname;
    ros::ServiceServer srv_simple_session;
    ros::ServiceServer srv_set_variable;
    ros::ServiceServer srv_get_variable;
    ros::ServiceServer srv_add_variables;
public:
    SimpleSession()
    {
	srv_simple_session = s_pmasternode->advertiseService("simple_session",&SimpleSession::startsession,this);// ,1);
        _sessionname = s_pmasternode->resolveName("simple_session");

        // advertise persistent services, the protocol for these differs!
        srv_set_variable = s_pmasternode->advertiseService("set_variable",&SimpleSession::set_variable,this); //,-1);
        srv_get_variable = s_pmasternode->advertiseService("get_variable",&SimpleSession::get_variable,this); //,-1);
        srv_add_variables = s_pmasternode->advertiseService("add_variables",&SimpleSession::add_variables,this); //,-1);
    }
    virtual ~SimpleSession()
    {
	srv_simple_session.shutdown();
	srv_set_variable.shutdown();
	srv_get_variable.shutdown();
	srv_add_variables.shutdown();
	
    }

    template <class MReq>
    SimpleSessionInstance* getstate(const MReq& req)
    {
        if( !req.__connection_header )
            return NULL;

        ros::M_string::const_iterator it = req.__connection_header->find(_sessionname);
        if( it == req.__connection_header->end() ) {
            ROS_WARN("failed to find header key %s",_sessionname.c_str());
            return NULL;
        }

        boost::mutex::scoped_lock lock(map_mutex);

        int sessionid = atoi(it->second.c_str());
        if( mapsessions.find(sessionid) == mapsessions.end() ) {
            ROS_WARN("failed to find session id %d", sessionid);
            return NULL;
        }
        return mapsessions[sessionid].get();
    }

    bool startsession(roscpp_sessions::simple_session::Request& req, roscpp_sessions::simple_session::Response& res) {
        if( req.sessionid ) {
            // destroy
            int success = mapsessions.erase(req.sessionid)>0;
            cout << "terminate session: " << req.sessionid << ", success: " << success << endl;
        }
        else {
            // start a new session with id
            int id = rand();
            ROS_ASSERT( mapsessions.find(id) == mapsessions.end() );
            mapsessions[id].reset(new SimpleSessionInstance());
            cout << "simple session " << id << " started with options " << req.options << endl;
            res.sessionid = id;
        }
        return true;
    }

    bool set_variable(roscpp_sessions::set_variable::Request& req,
                      roscpp_sessions::set_variable::Response& res)
    {
        SimpleSessionInstance* pinst = getstate(req);
        if( pinst == NULL )
            return false;
        pinst->set_variable(req.variable,req.value);
        return true;
    }

    bool get_variable(roscpp_sessions::get_variable::Request& req,
                      roscpp_sessions::get_variable::Response& res)
    {
        SimpleSessionInstance* pinst = getstate(req);
        if( pinst == NULL )
            return false;
        res.result = pinst->get_variable(req.variable);
        return true;
    }

    bool add_variables(roscpp_sessions::add_variables::Request& req,
                      roscpp_sessions::add_variables::Response& res)
    {
        SimpleSessionInstance* pinst = getstate(req);
        if( pinst == NULL )
            return false;
        pinst->add_variables(req.result, req.variable1,req.variable2);
        return true;
    }

private:
    map<int,boost::shared_ptr<SimpleSessionInstance> > mapsessions;
    boost::mutex map_mutex;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_session");
  
  s_pmasternode.reset(new ros::NodeHandle());
  if( !ros::master::check() )
      return 1;
  
  boost::shared_ptr<SimpleSession> server(new SimpleSession());
  ros::spin();

  server.reset();
  s_pmasternode.reset();  
  return 0;
}
