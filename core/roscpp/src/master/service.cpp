///////////////////////////////////////////////////////////////////////////////
// The roscpp package provides a c++ client library implementation for ROS.
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

#include <sstream>
#include <cstdlib>
#include "topic.h"
#include "service.h"
#include "master.h"

using namespace ros;

bool service::set_pub(process *p, const string &_pub_uri)
{
  if (pub && *pub == *p && pub_uri == _pub_uri)
    return false;
  pub = p;
  pub_uri = _pub_uri;
  return true;
}

bool service::add_sub(process *p)
{
  for (vector<process *>::iterator i = subs.begin(); i != subs.end(); ++i)
    if (**i == *p)
      return false;
  subs.push_back(p);
  return true;
}

bool service::remove_pub(process *p)
{
  if (!p || !pub)
    return false;
  if (*p == *pub)
  {
    pub = NULL;
    pub_uri = string();
    return true;
  }
  else
    return false;
}

bool service::remove_sub(process *p)
{
  for (vector<process *>::iterator i = subs.begin(); i != subs.end(); ++i)
    if (**i == *p)
    {
      subs.erase(i);
      return true;
    }
  return false; // never found it.
}

vector<string> service::sub_uris()
{
  vector<string> uris;
  for (vector<process *>::iterator i = subs.begin(); i != subs.end(); ++i)
    uris.push_back((*i)->uri);
  return uris;
}

int service::num_procs()
{
  return (pub ? 1 : 0) + subs.size();
}

bool service::has_pub(process *p)
{
  return (pub && *pub == *p ? true : false);
}

bool service::has_sub(process *p)
{
  for (vector<process *>::iterator i = subs.begin(); i != subs.end(); ++i)
    if (*i == p)
      return true;
  return false;
}

bool service::split_uri(const string &uri, string &host, int &port)
{
  // skip over http:// part if it's there
  string::size_type p = uri.find("http://");
  if (p != string::npos)
    host = string(uri.substr(p+7));
  else
    host = uri;
  // split out the port
  string::size_type colon_pos = host.find_first_of(":");
  if (colon_pos == string::npos)
    return false;
  string port_str = host.substr(colon_pos+1);
  string::size_type slash_pos = port_str.find_first_of("/");
  if (slash_pos != string::npos)
    port_str = port_str.erase(slash_pos);
  port = atoi(port_str.c_str());
  host = host.erase(colon_pos);
//  printf("uri = [%s] host = [%s] port = [%s] (%d)\n",
//    uri.c_str(), host.c_str(), port_str.c_str(), port);
  return true;
}

void service::send_service_updates()
{
  // call update RPC on all subscribers
  XmlRpcValue params, result;
  params[0] = string("master");
  params[1] = name;
  params[2] = pub_uri;

  vector<string> sub_uris_vec = sub_uris();
  for (vector<string>::iterator i  = sub_uris_vec.begin();
       i != sub_uris_vec.end(); ++i)
  {
    string sub_host;
    int sub_port;
    if (!split_uri(*i, sub_host, sub_port))
      g_master->log(FATAL, "woah! illegal URI: [%s]", (*i).c_str());
    XmlRpcClient client(sub_host.c_str(), sub_port, "/");
    try
    {
      client.execute("serverUpdate", params, result);
    }
    catch (exception const &e)
    {
      ostringstream oss;
      oss << e.what();
      g_master->log((log_level_t)(master::LOG_NOGUI | ERROR), 
                    "couldn't communicate with botnet process at %s: %s", 
                    (*i).c_str(), oss.str().c_str());
      // todo: drop them from our process table and all subscriber lists
    }
  }
}

