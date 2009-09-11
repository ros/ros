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

#include <cstdarg>
#include <cassert>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include "master.h"
#include "topic.h"
#include "service.h"
#include "ros/time.h"
#include "ros/common.h"

#if POSIX_TIMERS <= 0
#include <sys/time.h>
#endif

//#define REALLY_VERBOSE_LOG

using namespace ros;

master *ros::g_master = NULL;

namespace ros_rpc
{

XmlRpcValue response_str(int code, string msg, string response)
{
  XmlRpcValue v;
  v[0] = code;
  v[1] = msg;
  v[2] = response;
  return v;
}

XmlRpcValue response_int(int code, string msg, int response)
{
  XmlRpcValue v;
  v[0] = int(code);
  v[1] = msg;
  v[2] = response;
  return v;
}

XmlRpcValue response_bool(int code, string msg, bool response)
{
  XmlRpcValue v;
  v[0] = int(code);
  v[1] = msg;
  v[2] = XmlRpcValue(response);
  return v;
}

class shutdown : public XmlRpcServerMethod
{
public:
  shutdown(XmlRpcServer *s) : XmlRpcServerMethod("shutdown", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    g_master->log(INFO, "shutdown xmlrpc request");
    g_master->shutting_down = true;
    result = response_int(1, "", 0);
  }
};

class internalShutdown : public XmlRpcServerMethod
{
public:
  internalShutdown(XmlRpcServer *s) : XmlRpcServerMethod("_shutdown", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    g_master->log(INFO, "internal shutdown xmlrpc request");
    _server->exit();
    result = response_int(1, "", 0);
  }
};

class getTime : public XmlRpcServerMethod
{
public:
  getTime(XmlRpcServer *s) : XmlRpcServerMethod("getTime", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    XmlRpcValue a, t;
    result[0] = int(1);
    result[1] = string("");
    struct timespec ts;
#if POSIX_TIMERS > 0
    clock_gettime(CLOCK_REALTIME, &ts);
#else
    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    ts.tv_sec = timeofday.tv_sec;
    ts.tv_nsec = timeofday.tv_usec * 1000;
#endif
    t[0] = int(ts.tv_sec);
    t[1] = int(ts.tv_nsec);
    result[2] = t;
  }
};

class getPid : public XmlRpcServerMethod
{
public:
  getPid(XmlRpcServer *s) : XmlRpcServerMethod("getPid", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    string caller_id(params[0]);
    g_master->get_pid(caller_id, result);
  }
};

class lookupNode : public XmlRpcServerMethod
{
public:
  lookupNode(XmlRpcServer *s)
  : XmlRpcServerMethod("lookupNode", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    string caller_id(params[0]);
    string name(params[1]);
    g_master->lookup_node(name, result);
  }
};

class getSystemState : public XmlRpcServerMethod
{
public:
  getSystemState(XmlRpcServer *s)
  : XmlRpcServerMethod("getSystemState", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    g_master->get_system_state(result);
  }
};

class registerPublisher : public XmlRpcServerMethod
{
public:
  registerPublisher(XmlRpcServer *s) 
  : XmlRpcServerMethod("registerPublisher", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    string caller_id(params[0]);
    string topic(params[1]);
    string topic_type(params[2]);
    string caller_api(params[3]);
    g_master->register_publisher(caller_id, topic, topic_type, 
                                 caller_api, result);
  }
};

class registerService : public XmlRpcServerMethod
{
public:
  registerService(XmlRpcServer *s) 
  : XmlRpcServerMethod("registerService", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    string caller_id(params[0]);
    string serv_name(params[1]);
    string serv_api(params[2]);
    string caller_api(params[3]);
    g_master->register_service(caller_id, caller_api, serv_name, serv_api, 
                               result);
  }
};

class lookupService : public XmlRpcServerMethod
{
public:
  lookupService(XmlRpcServer *s)
  : XmlRpcServerMethod("lookupService", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    string caller_id(params[0]);
    string serv_name(params[1]);
    g_master->lookupService(caller_id, serv_name, result);
  }
};

class unregisterPublisher : public XmlRpcServerMethod
{
public:
  unregisterPublisher(XmlRpcServer *s) 
  : XmlRpcServerMethod("unregisterPublisher", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    string caller_id(params[0]);
    string topic(params[1]);
    string caller_api(params[2]);
    if (g_master->unregister_publisher(caller_id, topic, caller_api))
      result = response_int(1, "", 1);
    else
      result = response_int(1, "", 0); // not an error. just zero unregistered
  }
};

class unregisterService : public XmlRpcServerMethod
{
public:
  unregisterService(XmlRpcServer *s) 
  : XmlRpcServerMethod("unregisterService", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    string caller_id(params[0]);
    string serv_name(params[1]);
    string serv_api(params[2]);
    if (g_master->unregister_service(caller_id, serv_name, serv_api))
      result = response_int(1, "", 1);
    else
      result = response_int(1, "", 0); // not an error. just zero unregistered
  }
};

class registerSubscriber : public XmlRpcServerMethod
{
public:
  registerSubscriber(XmlRpcServer *s) 
  : XmlRpcServerMethod("registerSubscriber", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    g_master->registerSubscriber(params[0], params[1], params[2], 
                                  params[3], result);
  }
};

class unregisterSubscriber : public XmlRpcServerMethod
{
public:
  unregisterSubscriber(XmlRpcServer *s) 
  : XmlRpcServerMethod("unregisterSubscriber", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    if (g_master->unregisterSubscriber(params[0], params[1], params[2]))
      result = response_int(1, "", 1);
    else
      result = response_int(1, "", 0); // not an error. just zero unregistered
  }
};

class getPublishedTopics : public XmlRpcServerMethod
{
public:
  getPublishedTopics(XmlRpcServer *s) 
  : XmlRpcServerMethod("getPublishedTopics", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    g_master->getPublishedTopics(params[0], params[1], result);
  }
};

class setParam : public XmlRpcServerMethod
{
public:
  setParam(XmlRpcServer *s) : XmlRpcServerMethod("setParam", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    g_master->set_param(params[0], params[1], params[2]);
    result = response_int(1, "", 0);
  }
};

class getParam : public XmlRpcServerMethod
{
public:
  getParam(XmlRpcServer *s) : XmlRpcServerMethod("getParam", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    g_master->get_param(params[0], params[1], result);
  }
};

class getParamNames : public XmlRpcServerMethod
{
public:
  getParamNames(XmlRpcServer *s) : XmlRpcServerMethod("getParamNames", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    g_master->get_param_names(params[0], result);
  }
};

class hasParam : public XmlRpcServerMethod
{
public:
  hasParam(XmlRpcServer *s) : XmlRpcServerMethod("hasParam", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    g_master->has_param(params[0], params[1], result);
  }
};

class deleteParam : public XmlRpcServerMethod
{
public:
  deleteParam(XmlRpcServer *s) : XmlRpcServerMethod("deleteParam", s) { }
  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    g_master->delete_param(params[0], params[1], result);
  }
};

}

///////////////////////////////////////////////////////////////////////////
// global static functions
///////////////////////////////////////////////////////////////////////////
#if 0
static bool split_uri(const string &uri, string &host, int &port)
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
#endif

///////////////////////////////////////////////////////////////////////////
// master implementation
///////////////////////////////////////////////////////////////////////////

master::master(int port) : 
  shutting_down(false), xmlrpc_running(false),
  xmlrpc_port(port), _ok(true), gui_ok(false)
{
  g_master = this;
  pid = getpid();
  string log_file_name;
  char *ros_root_env = getenv("ROS_ROOT");
  if (ros_root_env)
    log_file_name = string(ros_root_env) + string("/log/");
  char pid_str[100];
  snprintf(pid_str, sizeof(pid_str), "%d", pid);
  log_file_name += string("botherder_") + string(pid_str) + string(".log");
  log_file = fopen(log_file_name.c_str(), "w");
  pthread_create(&xmlrpc_thread_handle, NULL, s_xmlrpc_thread_func, this);
  pthread_create(&update_thread_handle, NULL, s_publisher_updates_thread, this);
}

master::~master()
{
}

void master::log(log_level_t level, const char *format, ...)
{
  va_list args;
  va_start(args, format);
  fprintf(log_file, "%f ", Time::now().toSec());
  vfprintf(log_file, format, args);
  va_end(args);
  if (format[strlen(format)-1] != '\n')
    fprintf(log_file, "\n");
#ifdef REALLY_VERBOSE_LOG
  fprintf(log_file, "###########\n");
  for (vector<process *>::iterator p = procs.begin(); p != procs.end(); ++p)
  {
    fprintf(log_file, "process %s (%s)\n", (*p)->name.c_str(), 
            (*p)->uri.c_str());
  }
  for (vector<service *>::iterator s = services.begin(); 
       s != services.end(); ++s)
  {
    fprintf(log_file, "service %s: published by %s\n", (*s)->name.c_str(), 
            (*s)->pub ? (*s)->pub->name.c_str() : "(nobody)");
  }
  for (vector<topic *>::iterator t = topics.begin();
       t != topics.end(); t++)
  {
    fprintf(log_file, "topic %s\n pubs:\n", (*t)->name.c_str());
    for (vector<process *>::iterator p = (*t)->pubs.begin();
         p != (*t)->pubs.end(); ++p)
      fprintf(log_file, "  %s (%s)\n", (*p)->name.c_str(), (*p)->uri.c_str());
    fprintf(log_file, " subs:\n");
    for (vector<process *>::iterator p = (*t)->subs.begin();
         p != (*t)->subs.end(); ++p)
      fprintf(log_file, "  %s (%s)\n", (*p)->name.c_str(), (*p)->uri.c_str());
  }

  fprintf(log_file, "***********\n");
#endif
  fflush(log_file);
  if (gui_ok && !((uint32_t)level & LOG_NOGUI))
  {
    va_list args;
    va_start(args, format);
    log_console(level, format, args);
    va_end(args);
  }
  if (level == FATAL)
  {
    printf("WOAH! fatal error!\n");
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
    exit(1);
  }
}

void *master::s_xmlrpc_thread_func(void *parent)
{
  ((master *)parent)->xmlrpc_thread_func();
  return NULL;
}

void master::xmlrpc_thread_func()
{
  log(INFO, "enter xmlrpc thread");
  ros_rpc::shutdown             m_shutdown(&s);
  ros_rpc::internalShutdown     m_internalShutdown(&s);
  ros_rpc::getPid               m_getPid(&s);
  ros_rpc::getTime              m_getTime(&s);
  ros_rpc::getPublishedTopics   m_getPublishedTopics(&s);  
  ros_rpc::registerPublisher    m_registerPublisher(&s);
  ros_rpc::unregisterPublisher  m_unregisterPublisher(&s);
  ros_rpc::registerSubscriber   m_registerSubscriber(&s);
  ros_rpc::unregisterSubscriber m_unregisterSubscriber(&s);
  ros_rpc::registerService      m_registerService(&s);
  ros_rpc::unregisterService    m_unregisterService(&s);
  ros_rpc::lookupService        m_lookupService(&s);
  ros_rpc::setParam             m_setParam(&s);
  ros_rpc::getParam             m_getParam(&s);
  ros_rpc::getParamNames        m_getParamNames(&s);
  ros_rpc::hasParam             m_hasParam(&s);
  ros_rpc::deleteParam          m_deleteParam(&s);
  ros_rpc::lookupNode           m_lookupNode(&s);
  ros_rpc::getSystemState       m_getSystemState(&s);

  s.bindAndListen(xmlrpc_port);
  s.work(-1.0); // this means to work indefinitely until s.exit() is called
  _ok = false;
  log(INFO, "leave xmlrpc thread");
}

void master::self_destruct()
{
  log(INFO, "initiating self destruct sequence. the red light is blinking.");
  shutting_down = true;
  //xmlrpc_client_mutex.lock();
  XmlRpcClient c("127.0.0.1", xmlrpc_port, "/");
  XmlRpcValue result;
  if (c.execute("_shutdown", XmlRpcValue(string("myself")), result))
    log(INFO, "sent xmlrpc shutdown request to myself\n");
  else  
    log(FATAL, "error sending shutdown request to myself\n");
  pthread_join(xmlrpc_thread_handle, NULL);
  pthread_join(update_thread_handle, NULL);
}

bool master::unregister_publisher(const string &caller_id, 
                                  const string &topic_name,
                                  const string &caller_api)
{
  log(INFO, "unregisterPublisher: caller_id=%s topic=%s caller_api=%s",
      caller_id.c_str(), topic_name.c_str(), caller_api.c_str());
  topic *t = get_topic(topic_name);
  if (!t)
  {
    log(WARNING, "woah! couldn't find topic [%s] for unregistering [%s]",
        topic_name.c_str(), caller_id.c_str());
    return false;
  }
  process *p = get_process(caller_id, caller_api, false);
  if (!p)
  {
    log(WARNING, "woah! couldn't find process [%s] at [%s]\n", 
        caller_id.c_str(), caller_api.c_str());
    return false;
  }
  bool remove_ok = t->remove_pub(p);
  if (remove_ok)
    log(INFO, "process %s is no longer a publisher of %s",
        caller_id.c_str(), topic_name.c_str());
  else
    log(ERROR, "error removing process %s from publishing %s",
        caller_id.c_str(), topic_name.c_str());
  // delete the process reference if it doesn't do anything anymore
  if (!num_topics(p))
  {
    log(INFO, "process %s doesn't participate in any other topics. deleting.",
        caller_id.c_str());
    delete_process(p);
  }
  if (!t->num_procs())
  {
    log(INFO, "topic %s doesn't have any pubs or subs anymore. deleting.",
        topic_name.c_str());
    delete_topic(t);
  }
  else
    t->send_publisher_updates_async();
  refresh_gui();
  return remove_ok;
}

bool master::unregister_service(const string &caller_id, 
                                const string &serv_name,
                                const string &serv_api)
{
  log(INFO, "unregisterService: caller_id=%s serv_name=%s serv_api=%s",
      caller_id.c_str(), serv_name.c_str(), serv_api.c_str());
  service *s = get_service(serv_name, false, serv_api);
  if (!s)
    return false;
  process *p = get_process(caller_id);
  if (!p)
  {
    log(WARNING, "woah! couldn't find process [%s]\n", caller_id.c_str()); 
    return false;
  }
  bool remove_ok = s->remove_pub(p);
  if (remove_ok)
    log(INFO, "process %s is no longer the server of %s",
        caller_id.c_str(), serv_name.c_str());
  else
    log(ERROR, "error removing process %s from serving %s",
        caller_id.c_str(), serv_name.c_str());
  // delete the process reference if it doesn't do anything anymore
  if (!num_topics(p))
  {
    log(INFO, "process %s doesn't participate in any other topics. deleting.",
        caller_id.c_str());
    delete_process(p);
  }
  if (!s->num_procs())
  {
    log(INFO, "service %s doesn't have any pubs or subs anymore. deleting.",
        serv_name.c_str());
    delete_service(s);
  }
  else
    s->send_service_updates();
  refresh_gui();
  return remove_ok;
}

bool master::unregisterSubscriber(const string &caller_id, 
                                   const string &topic_name,
                                   const string &caller_api)
{
  log(INFO, "unregisterSubscriber: caller_id=%s topic=%s caller_api=%s",
      caller_id.c_str(), topic_name.c_str(), caller_api.c_str());
  topic *t = get_topic(topic_name);
  if (!t)
  {
    log(WARNING, "woah! couldn't find topic [%s] to unregister subscriber "
                 "[%s]", topic_name.c_str(), caller_id.c_str());
    return false;
  }
  process *p = get_process(caller_id, caller_api, false);
  if (!p)
  {
    log(WARNING, "woah! caller_id [%s] is not in the process table, yet it "
                 "requested an unsubscribe from topic [%s]\n", 
       caller_id.c_str(), topic_name.c_str());
    return false;
  }
  bool remove_ok = t->remove_sub(p);
  if (remove_ok)
    log(INFO, "process %s is no longer a subscriber of %s",
        caller_id.c_str(), topic_name.c_str());
  else
    log(ERROR, "error removing process %s from subscribing to %s",
        caller_id.c_str(), topic_name.c_str());
  // delete the process reference if it doesn't do anything anymore
  if (!num_topics(p))
  {
    log(INFO, "process %s doesn't participate in any other topics. deleting.",
        caller_id.c_str());
    delete_process(p);
  }
  if (!t->num_procs())
  {
    log(INFO, "topic %s doesn't have any pubs or subs anymore. deleting.",
        topic_name.c_str());
    delete_topic(t);
  }
  refresh_gui();
  return remove_ok;
}

void master::get_pid(const string &caller_id, XmlRpcValue &ret)
{
  log(INFO, "getPid: caller_id=%s", caller_id.c_str());
  ret = ros_rpc::response_int(1, string(""), (int)pid); 
}

void master::register_publisher(const string &caller_id, 
  const string &topic_name, const string &topic_type, 
  const string &caller_api, XmlRpcValue &ret)
{
  log(INFO, "registerPublisher: caller_id=%s topic=%s topic_type=%s " \
            "caller_api=%s", caller_id.c_str(), topic_name.c_str(), 
            topic_type.c_str(), caller_api.c_str());
  string err_msg;
  topic *t = get_topic(topic_name, topic_type, err_msg);
  if (t)
  {
    t->add_pub(get_process(caller_id, caller_api, true));
    log(INFO, "%s added as a publisher to topic %s which has type %s",
        caller_id.c_str(), topic_name.c_str(), topic_type.c_str());
    ret = ros_rpc::response_int(1, "", t->pubs.size()-1);
    t->send_publisher_updates_async();
  }
  else
    ret = ros_rpc::response_int(0, err_msg, 0);
  refresh_gui();
}

void master::register_service(const string &caller_id, const string &caller_api,
  const string &service_name, const string &service_api, XmlRpcValue &ret)
{
  log(INFO, "registerService: caller_id=%s caller_api=%s serv_name=%s "
            "serv_api=%s ", caller_id.c_str(), caller_api.c_str(),
            service_name.c_str(), service_api.c_str());
  string err_msg;
  service *s = get_service(service_name, true);
  if (s->pub)
  {
    log(INFO, "%s already had a service provider at %s. replacing it with "
              "the supplied URI.", service_name.c_str(), s->pub_uri.c_str());
    // todo: clean out the process table if we've just blown away
    // the only thing this guy was doing
  }
  s->pub = get_process(caller_id, caller_api, true);
  s->pub_uri = service_api;
  ret = ros_rpc::response_int(1, "", 1);
  s->send_service_updates();
  refresh_gui();
}

void master::lookupService(const string &caller_id, const string &serv_name,
                            XmlRpcValue &ret)
{
  // removing the log because this can get called a *lot* 
//  log(INFO, "lookupService: caller_id=%s serv_name=%s",
//      caller_id.c_str(), serv_name.c_str());
  service *s = get_service(serv_name);
  if (!s)
    ret = ros_rpc::response_str(0, "service not found", "");
  else
    ret = ros_rpc::response_str(1, "", s->pub_uri);
}

void master::registerSubscriber(const string &caller_id, 
  const string &topic_name, const string &topic_type, 
  const string &caller_api, XmlRpcValue &ret)
{
  log(INFO, "registerSubscriber: caller_id=%s topic=%s topic_type=%s " \
            "caller_api=%s", caller_id.c_str(), topic_name.c_str(), 
            topic_type.c_str(), caller_api.c_str());
  // see if we have this topic already
  string err_msg;
  topic *t = get_topic(topic_name, topic_type, err_msg);
  if (t)
  {
    t->add_sub(get_process(caller_id, caller_api, true));
    log(INFO, "%s added as a subscriber to topic %s which has type %s",
        caller_id.c_str(), topic_name.c_str(), topic_type.c_str());
    XmlRpcValue topic_pubs;
    topic_pubs.setSize(0); // be sure we at least send an empty array
    vector<string> uris = t->pub_uris();
    for (size_t idx = 0; idx < uris.size(); idx++)
      topic_pubs[int(idx)] = uris[idx];
    ret[0] = int(1);
    ret[1] = string("");
    ret[2] = topic_pubs;
  }
  else
    ret = ros_rpc::response_int(0, err_msg, 0);
}
 
void master::getPublishedTopics(const string &caller_id, 
  const string &subgraph, XmlRpcValue &ret)
{
  log(INFO, "get_published_topics: caller_id=%s subgraph=%s ", caller_id.c_str(), subgraph.c_str());
  XmlRpcValue Publications;
  Publications.setSize(0);
  int tc = 0;
  string searchstr = (subgraph[subgraph.size() -1] != '/') ? subgraph + '/' : subgraph;
  size_t size = searchstr.size();
  for (vector<topic *>::const_iterator i = topics.begin(); i != topics.end(); ++i)
  {
    topic *t = (*i);
    if (t->name.substr(0, size) == searchstr && t->pub_uris().size() > 0)
    {
      XmlRpcValue pair;
      pair[0] = (*i)->name;
      pair[1] = (*i)->datatype;
      Publications[tc++] = pair;
    }
  }
  ret[0] = int(1);
  ret[1] = "Returning all topics matching "+searchstr;  
  ret[2] = Publications;
}


process *master::get_process(string name)
{
  // this version of the function returns NULL if the process doesn't exist
  for (vector<process *>::iterator i = procs.begin(); i != procs.end(); ++i)
    if ((*i)->name == name)
      return *i;
  return NULL;
}

process *master::get_process(string name, string uri, bool enforce_integrity)
{
  if (!enforce_integrity) // don't try to clean up the system
  {
    for (vector<process *>::iterator i = procs.begin(); i != procs.end(); ++i)
      if ((*i)->name == name && (*i)->uri == uri)
        return *i;
    return NULL;
  }
  // ensure there is only one process by this name, and it's this guy. 
  // otherwise, send a nasty message to the old guy and blow it off the botnet
  string last_api;
  int num_found = 0;
  for (vector<process *>::iterator i = procs.begin(); i != procs.end(); ++i)
    if ((*i)->name == name)
    {
      num_found++;
      last_api = (*i)->uri;
    }
  if (!num_found)
  {
    // easy; it's legal to add this guy to the process table
    process *ptr; 
    procs.push_back(ptr = new process(name, uri));
    return ptr;
  }
  else if (num_found == 1 && last_api == uri)
  {
    // also easy; there's only one guy, and he has the correct uri
    for (vector<process *>::iterator i = procs.begin(); i != procs.end(); ++i)
      if ((*i)->name == name && (*i)->uri == uri)
        return *i; 
    ROS_BREAK(); // shouldn't ever get here
  }
  else // we have a problem.
  {
    // blow away duplicate guys, and update the related topics
    vector<topic *> affected_topics;
    vector<process *> dups;
    for (vector<process *>::iterator i = procs.begin(); i != procs.end(); )
      if ((*i)->name == name && (*i)->uri != uri)
      {
        log(ERROR, "process [%s] already exists on the system at URI [%s], "
            "which I am un-registering forcefully and replacing it with the "
            "version at URI [%s]", name.c_str(), (*i)->uri.c_str(), 
            uri.c_str());
        dups.push_back(*i);
        i = procs.erase(i);
      }
      else
        ++i;

    for (vector<process *>::iterator i = dups.begin(); i != dups.end(); ++i) 
    {
      for (vector<topic *>::iterator t = topics.begin(); t != topics.end(); ++t)
      {
        for (vector<process *>::iterator p = (*t)->pubs.begin(); 
             p != (*t)->pubs.end(); ++p)
        {
          if (*p == *i)
          {
            affected_topics.push_back(*t);
            (*t)->pubs.erase(p);
            break;
          }
        }
        for (vector<process *>::iterator p = (*t)->subs.begin();
             p != (*t)->subs.end(); ++p)
          if (*p == *i)
          {
            (*t)->subs.erase(p);
            break;
          }
      }
      for (vector<service *>::iterator s = services.begin(); s != services.end(); ++s)
      {
        if ((*s)->pub == *i)
        {
          log(INFO, "getting rid of stale service pointer for service [%s]\n", (*s)->name.c_str());
          (*s)->pub = NULL;
        }
      }

      string client_host;
      int client_port;
      if (!split_uri((*i)->uri, client_host, client_port))
        log(ERROR, "woah! bogus client URI: [%s]. aaaaa", (*i)->uri.c_str());
      else
      {
        XmlRpcClient c(client_host.c_str(), client_port, "/");
        XmlRpcValue args, result;
        args[0] = "master";
        ostringstream oss;
        oss << "Another node named [" << name << "] registered from URI ["
            << uri << "] and since you were the older registration, I "
               "am assuming that you have crashed or are otherwise "
               "toasted. Therefore, you are being shut down. BYE. "
               "If you had wanted to launch more than one instance of "
               "this node simultaneously, you need to give each instance "
               "a unique name on the command line, by supplying a remapping "
               "to the name given in the code, like this:\n  "
               "  ./my_node __name:=UNIQUE_STRING\n"
               "or you could have started them in different namespaces.\n";
        args[1] = oss.str();
        try
        {
          c.execute("shutdown", args, result);
        }
        catch(const exception &e)
        {
          ostringstream oss;
          oss << e.what();
          log(ERROR, "woah! exception trying to shut down duplicate node [%s] "
              "at [%s]: [%s]\n", name.c_str(), (*i)->uri.c_str(), 
              oss.str().c_str());
        }
      }
      delete (*i);
    }

    for (vector<topic *>::iterator t = affected_topics.begin();
         t != affected_topics.end(); ++t)
      (*t)->send_publisher_updates_async();

    // now that we've cleaned up the duplicated, find or create the real one
    process p(name, uri);
    for (vector<process *>::iterator i = procs.begin(); i != procs.end(); ++i)
      if (**i == p)
        return *i;
    process *ptr; 
    procs.push_back(ptr = new process(name, uri));
    return ptr;
  }
  
  return NULL;
}

int master::num_topics(process *p)
{
  int count = 0;
  for (vector<topic *>::iterator i = topics.begin(); i != topics.end(); ++i)
    if ((*i)->has_pub(p) || (*i)->has_sub(p))
      count++;
  for (vector<service *>::iterator i = services.begin(); 
       i != services.end(); ++i)
    if ((*i)->has_pub(p) || (*i)->has_sub(p))
      count++;
  return count;
}

bool master::delete_topic(topic *t)
{
  for (vector<topic *>::iterator i = topics.begin(); i != topics.end(); ++i)
    if (*i == t)
    {
      topics.erase(i);
      delete t;
      return true;
    }
  return false;
}

bool master::delete_service(service *s)
{
  for (vector<service *>::iterator i = services.begin(); 
       i != services.end(); ++i)
    if (*i == s)
    {
      services.erase(i);
      delete s;
      return true;
    }
  return false;
}

bool master::delete_process(process *p)
{
  // make sure this process isn't referenced by any other data structures
  for (vector<service *>::iterator s = services.begin(); s != services.end(); ++s)
  {
    if ((*s)->pub == p)
      assert(0);
  }
  for (vector<topic *>::iterator t = topics.begin(); t != topics.end(); ++t)
  {
    if ((*t)->has_pub(p))
      assert(0);
    if ((*t)->has_sub(p))
      assert(0);
  }
  for (vector<process *>::iterator i = procs.begin(); i != procs.end(); ++i)
    if (*i == p)
    {
      procs.erase(i);
      delete p;
      return true;
    }
  return false;
}

topic *master::get_topic(const string &topic_name)
{
  for (vector<topic *>::iterator i = topics.begin(); i != topics.end(); ++i)
    if ((*i)->name == topic_name)
      return *i;
  return NULL;
}

service *master::get_service(const string &service_name, bool should_create,
                             const string &service_api)
{
  for (vector<service *>::iterator i = services.begin(); 
       i != services.end(); ++i)
    if ((*i)->name == service_name && 
        (service_api.length() == 0 || (*i)->pub_uri == service_api))
      return *i;
  if (should_create)
  {
    service *s = new service(service_name);
    services.push_back(s);
    return s;
  }
  else
    return NULL;
}

topic *master::get_topic(const string &topic_name, const string &topic_type,
                 string &err_msg)
{
  for (vector<topic *>::iterator i = topics.begin(); i != topics.end(); ++i)
  {
    if ((*i)->name == topic_name)
    {
      if ((*i)->datatype == topic_type || topic_type == string("*"))
        return *i;
      else if ((*i)->datatype == string("*"))
      {
        (*i)->datatype = topic_type; // assume the first non-wildcard type
        return *i;
      }
      else
      {
        char err_msg_cstr[1000];
        snprintf(err_msg_cstr, sizeof(err_msg_cstr), "type error: referred " \
          "to topic %s as having type %s, but that topic already has " \
          "type %s",
          topic_name.c_str(), topic_type.c_str(), (*i)->name.c_str());
        log(ERROR, err_msg_cstr);
        err_msg = string(err_msg_cstr);
        return NULL;
      }
    }
  }
  topic *t = new topic(topic_name, topic_type);
  topics.push_back(t);
  return t;
}

void master::set_param(const string &caller_id, const string &key, 
                       const XmlRpcValue &v)
{
  log(INFO, "caller id = %s param key = %s\n", caller_id.c_str(), key.c_str());
  // find namespace of param
  string full_name = key;
  if (key[0] != '/')
  {
    string::size_type last_slash = caller_id.find_last_of("/");
    if (last_slash == string::npos)
      log(FATAL, "woah! caller_id %s was not a globally-referenced identifier",
          caller_id.c_str());
    string ns = caller_id.substr(0, last_slash+1);
    full_name = ns + key;
  }
  log(INFO, "setting param %s", full_name.c_str());
  params[full_name] = v;
  refresh_gui();
}
  
void master::get_param(const string &caller_id, const string &key, 
                       XmlRpcValue &ret)
{
  log(INFO, "getParam: caller_id = %s key = %s\n", 
      caller_id.c_str(), key.c_str());
  // if we decide we want it, could do a hierarchical lookup here
  string full_name = key;
  if (key[0] != '/')
  {
    string::size_type last_slash = caller_id.find_last_of("/");
    if (last_slash == string::npos)
      log(FATAL, "woah! caller_id %s was not a globally-referenced identifier",
          caller_id.c_str());
    string ns = caller_id.substr(0, last_slash+1);
    full_name = ns + key;
  }
  log(INFO, "getting param %s", full_name.c_str());
  if (params.count(full_name) == 0)
  {
    log(ERROR, "parameter %s was not defined", full_name.c_str());
    ret = ros_rpc::response_int(-1, "parameter not found", 0);
  }
  else
  {
    ret[0] = int(1);
    ret[1] = string();
    ret[2] = params[full_name];
  }
}

void master::has_param(const string &caller_id, const string &key, 
                       XmlRpcValue &ret)
{
  log(INFO, "hasParam: caller_id = %s key = %s\n", 
      caller_id.c_str(), key.c_str());
  // if we decide we want it, could do a hierarchical lookup here
  string full_name = key;
  if (key[0] != '/')
  {
    string::size_type last_slash = caller_id.find_last_of("/");
    if (last_slash == string::npos)
      log(FATAL, "woah! caller_id %s was not a globally-referenced identifier",
          caller_id.c_str());
    string ns = caller_id.substr(0, last_slash+1);
    full_name = ns + key;
  }
  log(INFO, "checking for param %s", full_name.c_str());
  ostringstream oss;
  XmlRpcValue v = ros_rpc::response_bool(1,"",false);
  oss << v.toXml();
  if (params.count(full_name) == 0)
    ret = ros_rpc::response_bool(1, "", false);
  else
    ret = ros_rpc::response_bool(1, "", true);
}

void master::delete_param(const string &caller_id, const string &key, 
                          XmlRpcValue &ret)
{
  log(INFO, "deleteParam: caller_id = %s key = %s\n", 
      caller_id.c_str(), key.c_str());
  // if we decide we want it, could do a hierarchical lookup here
  string full_name = key;
  if (key[0] != '/')
  {
    string::size_type last_slash = caller_id.find_last_of("/");
    if (last_slash == string::npos)
      log(FATAL, "woah! caller_id %s was not a globally-referenced identifier",
          caller_id.c_str());
    string ns = caller_id.substr(0, last_slash+1);
    full_name = ns + key;
  }
  log(INFO, "deleting param %s", full_name.c_str());
  ostringstream oss;
  XmlRpcValue v = ros_rpc::response_bool(1,"",false);
  oss << v.toXml();
  if (params.count(full_name) == 0)
    ret = ros_rpc::response_int(1, "", 0);
  else
  {
    params.erase(full_name);
    ret = ros_rpc::response_int(1, "", 1);
  }
  refresh_gui();
}

void *master::s_publisher_updates_thread(void *param)
{
  master *m = (master *)param;
  while (!m->shutting_down)
  {
    usleep(100000); // send updates at max 10 hz
    if (m->shutting_down)
      break;
    m->pub_updates_vec_mutex.lock();
    if (m->pub_updates.size() == 0)
    {
      m->pub_updates_vec_mutex.unlock();
      continue; // spin around
    }
    // build an update queue of the last messages for each topic
    vector<pub_update> latest;
    for (vector<pub_update>::reverse_iterator i = m->pub_updates.rbegin();
         i != m->pub_updates.rend(); ++i)
    {
      // see if we already have an entry for this guy in our 'latest' table
      bool found = false;
      for (vector<pub_update>::iterator j = latest.begin(); 
           !found && j != latest.end(); ++j)
        if (i->name == j->name)
          found = true; // don't send duplicate messages
      if (!found)
        latest.push_back(*i);
    }
    m->pub_updates.clear();
    m->pub_updates_vec_mutex.unlock(); // we're done with it; we have our copy

    for (vector<pub_update>::iterator t = latest.begin();  // t = topic 
         t != latest.end(); ++t)
    {
      // call update RPC on all subscribers
      XmlRpcValue xml_pub_uris, params, result;
      xml_pub_uris.setSize(0); // force it to be an array
      for (size_t i = 0; i < t->pub_uris_vec.size(); i++)
        xml_pub_uris[i] = t->pub_uris_vec[i];
      params[0] = string("master");
      params[1] = t->name; // the topic name
      params[2] = xml_pub_uris;
      for (vector<string>::iterator i  = t->sub_uris_vec.begin();
           i != t->sub_uris_vec.end(); ++i)
      {
        string sub_host;
        int sub_port;
        if (!split_uri(*i, sub_host, sub_port))
          g_master->log(FATAL, "woah! illegal URI: [%s]", (*i).c_str());
        XmlRpcClient *c = new XmlRpcClient(sub_host.c_str(), sub_port, "/");
        c->setKeepOpen(false);
        try
        {
          c->execute("publisherUpdate", params, result);
        }
        catch (exception const &e)
        {
          ostringstream oss;
          oss << e.what();
          g_master->log((log_level_t)(master::LOG_NOGUI | ERROR), 
                        "(topic update) couldn't communicate w/process "
                        "at %s: %s", (*i).c_str(), oss.str().c_str());
            // todo: drop them from our process table and all subscriber lists
        }
        delete c;
      }
    }
  }
  return NULL;
}

bool master::split_uri(const string &uri, string &host, int &port)
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
  
void master::enqueue_pub_update(pub_update pu)
{
  pub_updates_vec_mutex.lock();
  pub_updates.push_back(pu);
  pub_updates_vec_mutex.unlock();
}

void master::lookup_node(const string &name, XmlRpcValue &ret)
{
  log(INFO, "lookup_node: name=%s", name.c_str());
  process *p = get_process(name);
  if (p)
    ret = ros_rpc::response_str(1, string(""), p->uri);
  else
    ret = ros_rpc::response_str(-1, string("couldn't find node ") + name,
                                string(""));
}

void master::get_system_state(XmlRpcValue &ret)
{
  ret[0] = 1;
  ret[1] = string("");
  XmlRpcValue state;
  XmlRpcValue pubs;
  XmlRpcValue subs;
  XmlRpcValue srvs;
  pubs.setSize(0);
  subs.setSize(0);
  srvs.setSize(0);

  for(vector<topic*>::iterator t = topics.begin();
      t != topics.end();
      ++t)
  {
    XmlRpcValue sub, sub_data;
    sub[0] = (*t)->name;
    sub_data.setSize(0);
    for(vector<process*>::iterator p = (*t)->subs.begin();
        p != (*t)->subs.end();
        ++p)
    {
      sub_data[sub_data.size()] = (*p)->name.c_str();
    }
    sub[1] = sub_data;
    subs[subs.size()] = sub;

    XmlRpcValue pub, pub_data;
    pub[0] = (*t)->name;
    pub_data.setSize(0);
    for(vector<process*>::iterator p = (*t)->pubs.begin();
        p != (*t)->pubs.end();
        ++p)
    {
      pub_data[pub_data.size()] = (*p)->name.c_str();
    }
    pub[1] = pub_data;
    pubs[pubs.size()] = pub;
  }

  for(vector<service *>::iterator s = services.begin();
      s != services.end();
      ++s)
  {
    XmlRpcValue srv, srv_data;
    srv[0] = (*s)->name;
    srv_data[0] = (*s)->pub_uri;
    srv[1] = srv_data;

    srvs[srvs.size()] = srv;
  }
  state[0] = pubs;
  state[1] = subs;
  state[2] = srvs;

  ret[2] = state;

}

void master::get_param_names(const string &caller_id, XmlRpcValue &ret)
{
  XmlRpcValue names;
  names.setSize(0); // be sure we at least send an empty array  
  int i = 0;
  for (map<string, XmlRpcValue>::iterator p = params.begin();
       p != params.end(); ++p)
    names[i++] = p->first;
  ret[0] = 1;
  ret[1] = "";
  ret[2] = names;
}
