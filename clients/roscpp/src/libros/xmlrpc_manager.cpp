/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include "ros/xmlrpc_manager.h"
#include "ros/network.h"
#include "ros/param.h"
#include "ros/assert.h"
#include "ros/common.h"
#include "ros/file_log.h"
#include "ros/io.h"

using namespace XmlRpc;

namespace ros
{

namespace xmlrpc
{
XmlRpc::XmlRpcValue responseStr(int code, const std::string& msg, const std::string& response)
{
  XmlRpc::XmlRpcValue v;
  v[0] = code;
  v[1] = msg;
  v[2] = response;
  return v;
}

XmlRpc::XmlRpcValue responseInt(int code, const std::string& msg, int response)
{
  XmlRpc::XmlRpcValue v;
  v[0] = int(code);
  v[1] = msg;
  v[2] = response;
  return v;
}

XmlRpc::XmlRpcValue responseBool(int code, const std::string& msg, bool response)
{
  XmlRpc::XmlRpcValue v;
  v[0] = int(code);
  v[1] = msg;
  v[2] = XmlRpc::XmlRpcValue(response);
  return v;
}
}

class XMLRPCCallWrapper : public XmlRpcServerMethod
{
public:
  XMLRPCCallWrapper(const std::string& function_name, const XMLRPCFunc& cb, XmlRpcServer *s)
  : XmlRpcServerMethod(function_name, s)
  , name_(function_name)
  , func_(cb)
  { }

  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    func_(params, result);
  }

private:
  std::string name_;
  XMLRPCFunc func_;
};

void getPid(const XmlRpcValue& params, XmlRpcValue& result)
{
  result = xmlrpc::responseInt(1, "", (int)getpid());
}

const ros::WallDuration CachedXmlRpcClient::s_zombie_time_(30.0); // reap after 30 seconds

XMLRPCManagerPtr g_xmlrpc_manager;
boost::mutex g_xmlrpc_manager_mutex;
const XMLRPCManagerPtr& XMLRPCManager::instance()
{
  if (!g_xmlrpc_manager)
  {
    boost::mutex::scoped_lock lock(g_xmlrpc_manager_mutex);
    if (!g_xmlrpc_manager)
    {
      g_xmlrpc_manager.reset(new XMLRPCManager);
    }
  }

  return g_xmlrpc_manager;
}

XMLRPCManager::XMLRPCManager()
: port_(0)
, shutting_down_(false)
, unbind_requested_(false)
{
}

XMLRPCManager::~XMLRPCManager()
{
  shutdown();
}

void XMLRPCManager::start()
{
  shutting_down_ = false;
  port_ = 0;
  bind("getPid", getPid);

  bool bound = server_.bindAndListen(0);
  (void) bound;
  ROS_ASSERT(bound);
  port_ = server_.get_port();
  ROS_ASSERT(port_ != 0);

  std::stringstream ss;
  ss << "http://" << network::getHost() << ":" << port_ << "/";
  uri_ = ss.str();

  server_thread_ = boost::thread(boost::bind(&XMLRPCManager::serverThreadFunc, this));
}

void XMLRPCManager::shutdown()
{
  if (shutting_down_)
  {
    return;
  }

  shutting_down_ = true;
  server_thread_.join();

  server_.close();

  // kill the last few clients that were started in the shutdown process
  for (V_CachedXmlRpcClient::iterator i = clients_.begin();
       i != clients_.end(); ++i)
  {
    for (int wait_count = 0; i->in_use_ && wait_count < 10; wait_count++)
    {
      ROSCPP_LOG_DEBUG("waiting for xmlrpc connection to finish...");
      ros::WallDuration(0.01).sleep();
    }

    i->client_->close();
    delete i->client_;
  }

  clients_.clear();

  boost::mutex::scoped_lock lock(functions_mutex_);
  functions_.clear();

  {
    S_ASyncXMLRPCConnection::iterator it = connections_.begin();
    S_ASyncXMLRPCConnection::iterator end = connections_.end();
    for (; it != end; ++it)
    {
      (*it)->removeFromDispatch(server_.get_dispatch());
    }
  }

  connections_.clear();

  {
    boost::mutex::scoped_lock lock(added_connections_mutex_);
    added_connections_.clear();
  }

  {
    boost::mutex::scoped_lock lock(removed_connections_mutex_);
    removed_connections_.clear();
  }
}

bool XMLRPCManager::validateXmlrpcResponse(const std::string& method, XmlRpcValue &response,
                                    XmlRpcValue &payload)
{
  if (response.getType() != XmlRpcValue::TypeArray)
  {
    ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return an array",
        method.c_str());
    return false;
  }
  if (response.size() != 3)
  {
    ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return a 3-element array",
        method.c_str());
    return false;
  }
  if (response[0].getType() != XmlRpcValue::TypeInt)
  {
    ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return a int as the 1st element",
        method.c_str());
    return false;
  }
  int status_code = response[0];
  if (response[1].getType() != XmlRpcValue::TypeString)
  {
    ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return a string as the 2nd element",
        method.c_str());
    return false;
  }
  std::string status_string = response[1];
  if (status_code != 1)
  {
    ROSCPP_LOG_DEBUG("XML-RPC call [%s] returned an error (%d): [%s]",
        method.c_str(), status_code, status_string.c_str());
    return false;
  }
  payload = response[2];
  return true;
}

void XMLRPCManager::serverThreadFunc()
{
  disableAllSignalsInThisThread();

  while(!shutting_down_)
  {
    {
      boost::mutex::scoped_lock lock(added_connections_mutex_);
      S_ASyncXMLRPCConnection::iterator it = added_connections_.begin();
      S_ASyncXMLRPCConnection::iterator end = added_connections_.end();
      for (; it != end; ++it)
      {
        (*it)->addToDispatch(server_.get_dispatch());
        connections_.insert(*it);
      }

      added_connections_.clear();
    }

    // Update the XMLRPC server, blocking for at most 100ms in select()
    {
      boost::mutex::scoped_lock lock(functions_mutex_);
      server_.work(0.1);
    }

    while (unbind_requested_)
    {
      WallDuration(0.01).sleep();
    }

    if (shutting_down_)
    {
      return;
    }

    {
      S_ASyncXMLRPCConnection::iterator it = connections_.begin();
      S_ASyncXMLRPCConnection::iterator end = connections_.end();
      for (; it != end; ++it)
      {
        if ((*it)->check())
        {
          removeASyncConnection(*it);
        }
      }
    }

    {
      boost::mutex::scoped_lock lock(removed_connections_mutex_);
      S_ASyncXMLRPCConnection::iterator it = removed_connections_.begin();
      S_ASyncXMLRPCConnection::iterator end = removed_connections_.end();
      for (; it != end; ++it)
      {
        (*it)->removeFromDispatch(server_.get_dispatch());
        connections_.erase(*it);
      }

      removed_connections_.clear();
    }
  }
}

XmlRpcClient* XMLRPCManager::getXMLRPCClient(const std::string &host, const int port, const std::string &uri)
{
  // go through our vector of clients and grab the first available one
  XmlRpcClient *c = NULL;

  boost::mutex::scoped_lock lock(clients_mutex_);

  for (V_CachedXmlRpcClient::iterator i = clients_.begin();
       !c && i != clients_.end(); )
  {
    if (!i->in_use_)
    {
      // see where it's pointing
      if (i->client_->getHost() == host &&
          i->client_->getPort() == port &&
          i->client_->getUri()  == uri)
      {
        // hooray, it's pointing at our destination. re-use it.
        c = i->client_;
        i->in_use_ = true;
        i->last_use_time_ = WallTime::now();
        break;
      }
      else if (i->last_use_time_ + CachedXmlRpcClient::s_zombie_time_ < WallTime::now())
      {
        // toast this guy. he's dead and nobody is reusing him.
        delete i->client_;
        i = clients_.erase(i);
      }
      else
      {
        ++i; // move along. this guy isn't dead yet.
      }
    }
    else
    {
      ++i;
    }
  }

  if (!c)
  {
    // allocate a new one
    c = new XmlRpcClient(host.c_str(), port, uri.c_str());
    CachedXmlRpcClient mc(c);
    mc.in_use_ = true;
    mc.last_use_time_ = WallTime::now();
    clients_.push_back(mc);
    //ROS_INFO("%d xmlrpc clients allocated\n", xmlrpc_clients.size());
  }
  // ONUS IS ON THE RECEIVER TO UNSET THE IN_USE FLAG
  // by calling releaseXMLRPCClient
  return c;
}

void XMLRPCManager::releaseXMLRPCClient(XmlRpcClient *c)
{
  boost::mutex::scoped_lock lock(clients_mutex_);

  for (V_CachedXmlRpcClient::iterator i = clients_.begin();
       i != clients_.end(); ++i)
  {
    if (c == i->client_)
    {
      i->in_use_ = false;
      break;
    }
  }
}

void XMLRPCManager::addASyncConnection(const ASyncXMLRPCConnectionPtr& conn)
{
  boost::mutex::scoped_lock lock(added_connections_mutex_);
  added_connections_.insert(conn);
}

void XMLRPCManager::removeASyncConnection(const ASyncXMLRPCConnectionPtr& conn)
{
  boost::mutex::scoped_lock lock(removed_connections_mutex_);
  removed_connections_.insert(conn);
}

bool XMLRPCManager::bind(const std::string& function_name, const XMLRPCFunc& cb)
{
  boost::mutex::scoped_lock lock(functions_mutex_);
  if (functions_.find(function_name) != functions_.end())
  {
    return false;
  }

  FunctionInfo info;
  info.name = function_name;
  info.function = cb;
  info.wrapper.reset(new XMLRPCCallWrapper(function_name, cb, &server_));
  functions_[function_name] = info;

  return true;
}

void XMLRPCManager::unbind(const std::string& function_name)
{
  unbind_requested_ = true;
  boost::mutex::scoped_lock lock(functions_mutex_);
  functions_.erase(function_name);
  unbind_requested_ = false;
}

} // namespace ros
