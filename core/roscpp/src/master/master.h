///////////////////////////////////////////////////////////////////////////////
// The roscpp package provides a c++ implementation for ROS.
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

#ifndef ROS_MASTER_H
#define ROS_MASTER_H

#include <string>
#include <pthread.h>
#include <stdarg.h>
#include <vector>
#include <cstdio>
#include "XmlRpc.h"
#include <boost/thread/mutex.hpp>
#include "ros/common.h"

using namespace std;
using namespace XmlRpc;

// This is defined as ros::log_level_t in ros/common.h
enum log_level_t { INFO=0, DEBUG=1, WARNING=2, ERROR=3, FATAL=4 };


namespace ros
{

class topic;
class process;
class service;

class pub_update
{
  public:
    string name; /** topic name */
    vector<string> sub_uris_vec; /** subscribers */
    vector<string> pub_uris_vec; /** publishers  */
};

class master
{
public:
  const static uint32_t LOG_NOGUI = 0x80000000;
  master(int port);
  virtual ~master();
  inline bool ok() { return _ok; }
  void self_destruct(); // everything in scifi has this

  void get_pid(const string &caller_id, XmlRpcValue &ret);
  void lookup_node(const string &name, XmlRpcValue &ret);
  void get_system_state(XmlRpcValue &ret);

  void register_publisher(const string &caller_id, const string &topic,
                          const string &topic_type, const string &caller_api,
                          XmlRpcValue &ret);
  bool unregister_publisher(const string &caller_id, const string &topic,
                            const string &caller_api);

  void register_service(const string &caller_id, const string &caller_api,
                        const string &serv_name, const string &serv_api,
                        XmlRpcValue &ret);
  bool unregister_service(const string &caller_id, const string &serv_name,
                          const string &serv_api);
  void lookupService(const string &caller_id, const string &serv_name,
                      XmlRpcValue &ret);

  void registerSubscriber(const string &caller_id, const string &topic,
                           const string &topic_type, const string &caller_api,
                           XmlRpcValue &ret);
  bool unregisterSubscriber(const string &caller_id, const string &topic,
                             const string &caller_api);

  void getPublishedTopics(const string &caller_id,
                            const string &subgraph, XmlRpcValue &ret);

  void get_param_names(const string &caller_id, XmlRpcValue &ret);

  void log(log_level_t level, const char *format, ...);

  void set_param(const string &caller_id, const string &key,
                 const XmlRpcValue &v);
  void get_param(const string &caller_id, const string &key,
                 XmlRpcValue &ret);
  void has_param(const string &caller_id, const string &key,
                 XmlRpcValue &ret);
  void delete_param(const string &caller_id, const string &key,
                    XmlRpcValue &ret);

  virtual void spin() = 0;
  boost::mutex send_publisher_updates_mutex;
  void enqueue_pub_update(pub_update pu);

  bool shutting_down;

protected:
  virtual void log_console(log_level_t level, const char *format,
                           va_list args) = 0;
  void refresh_gui() { if (gui_ok) _refresh_gui(); }
  virtual void _refresh_gui() = 0;

  FILE *log_file;
  pthread_t xmlrpc_thread_handle, update_thread_handle;
  bool xmlrpc_running;
  int xmlrpc_port;
  bool _ok, gui_ok;
  pid_t pid;
  vector<process *> procs;
  vector<topic *> topics;
  vector<service *> services;
  topic *get_topic(const string &topic_name, const string &topic_type,
                   string &err_msg);
  topic *get_topic(const string &topic_name);
  bool delete_topic(topic *t);
  bool delete_process(process *t);
  service *get_service(const string &service_name, bool should_create = false,
                       const string &serv_api = "");
  bool delete_service(service *s);

  static void *s_xmlrpc_thread_func(void *parent);
  void xmlrpc_thread_func();
  process *get_process(string name, string uri, bool enforce_integrity);
  process *get_process(string name);
  int num_topics(process *p);
  map<string, XmlRpcValue> params;
  XmlRpcServer s;
  boost::mutex pub_updates_vec_mutex;
  vector<pub_update> pub_updates;
  static bool split_uri(const string &uri, string &host, int &port);
  static void *s_publisher_updates_thread(void *params);
};

extern master *g_master;

}

#endif

