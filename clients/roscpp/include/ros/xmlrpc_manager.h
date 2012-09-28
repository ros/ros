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

#ifndef ROSCPP_XMLRPC_MANAGER_H
#define ROSCPP_XMLRPC_MANAGER_H

#include <string>
#include <set>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "common.h"
#include "XmlRpc.h"

#include <ros/time.h>


namespace ros
{

/**
 * \brief internal
 */
namespace xmlrpc
{
XmlRpc::XmlRpcValue responseStr(int code, const std::string& msg, const std::string& response);
XmlRpc::XmlRpcValue responseInt(int code, const std::string& msg, int response);
XmlRpc::XmlRpcValue responseBool(int code, const std::string& msg, bool response);
}

class XMLRPCCallWrapper;
typedef boost::shared_ptr<XMLRPCCallWrapper> XMLRPCCallWrapperPtr;

class ROSCPP_DECL ASyncXMLRPCConnection : public boost::enable_shared_from_this<ASyncXMLRPCConnection>
{
public:
  virtual ~ASyncXMLRPCConnection() {}

  virtual void addToDispatch(XmlRpc::XmlRpcDispatch* disp) = 0;
  virtual void removeFromDispatch(XmlRpc::XmlRpcDispatch* disp) = 0;

  virtual bool check() = 0;
};
typedef boost::shared_ptr<ASyncXMLRPCConnection> ASyncXMLRPCConnectionPtr;
typedef std::set<ASyncXMLRPCConnectionPtr> S_ASyncXMLRPCConnection;

class ROSCPP_DECL CachedXmlRpcClient
{
public:
  CachedXmlRpcClient(XmlRpc::XmlRpcClient *c)
  : in_use_(false)
  , client_(c)
  {
  }

  bool in_use_;
  ros::WallTime last_use_time_; // for reaping
  XmlRpc::XmlRpcClient* client_;

  static const ros::WallDuration s_zombie_time_; // how long before it is toasted
};

class XMLRPCManager;
typedef boost::shared_ptr<XMLRPCManager> XMLRPCManagerPtr;

typedef boost::function<void(XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&)> XMLRPCFunc;

class ROSCPP_DECL XMLRPCManager
{
public:
  static const XMLRPCManagerPtr& instance();

  XMLRPCManager();
  ~XMLRPCManager();

  /** @brief Validate an XML/RPC response
   *
   * @param method The RPC method that was invoked.
   * @param response The resonse that was received.
   * @param payload The payload that was received.
   *
   * @return true if validation succeeds, false otherwise.
   *
   * @todo Consider making this private.
   */
  bool validateXmlrpcResponse(const std::string& method, 
			      XmlRpc::XmlRpcValue &response, XmlRpc::XmlRpcValue &payload);

  /**
   * @brief Get the xmlrpc server URI of this node
   */
  inline const std::string& getServerURI() const { return uri_; }
  inline uint32_t getServerPort() const { return port_; }

  XmlRpc::XmlRpcClient* getXMLRPCClient(const std::string& host, const int port, const std::string& uri);
  void releaseXMLRPCClient(XmlRpc::XmlRpcClient* c);

  void addASyncConnection(const ASyncXMLRPCConnectionPtr& conn);
  void removeASyncConnection(const ASyncXMLRPCConnectionPtr& conn);

  bool bind(const std::string& function_name, const XMLRPCFunc& cb);
  void unbind(const std::string& function_name);

  void start();
  void shutdown();

  bool isShuttingDown() { return shutting_down_; }

private:
  void serverThreadFunc();

  std::string uri_;
  int port_;
  boost::thread server_thread_;

#if defined(__APPLE__)
  // OSX has problems with lots of concurrent xmlrpc calls
  boost::mutex xmlrpc_call_mutex_;
#endif
  XmlRpc::XmlRpcServer server_;
  typedef std::vector<CachedXmlRpcClient> V_CachedXmlRpcClient;
  V_CachedXmlRpcClient clients_;
  boost::mutex clients_mutex_;

  bool shutting_down_;

  ros::WallDuration master_retry_timeout_;

  S_ASyncXMLRPCConnection added_connections_;
  boost::mutex added_connections_mutex_;
  S_ASyncXMLRPCConnection removed_connections_;
  boost::mutex removed_connections_mutex_;

  S_ASyncXMLRPCConnection connections_;


  struct FunctionInfo
  {
    std::string name;
    XMLRPCFunc function;
    XMLRPCCallWrapperPtr wrapper;
  };
  typedef std::map<std::string, FunctionInfo> M_StringToFuncInfo;
  boost::mutex functions_mutex_;
  M_StringToFuncInfo functions_;

  volatile bool unbind_requested_;
};

}

#endif
