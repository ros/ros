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

#include <cstdarg>
#include <sstream>
#include <cerrno>
#include <algorithm>

#include <sys/poll.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <cstdlib>
#include <cstring>

#include "ros/node.h"
#include "ros/subscription.h"
#include "ros/header.h"
#include "ros/time.h"
#include "ros/rosout_appender.h"
#include "ros/connection.h"
#include "ros/publication.h"
#include "ros/publisher.h"
#include "ros/service_server.h"
#include "ros/service_client.h"
#include "ros/service_link.h"
#include "ros/transport/transport_tcp.h"

#include "log4cxx/rollingfileappender.h"
#include "log4cxx/patternlayout.h"

#include <boost/bind.hpp>
#include <boost/thread/thread_time.hpp>

#include <config.h>
#ifdef HAVE_IFADDRS_H
  #include <ifaddrs.h>
#endif

using namespace std;
using namespace ros;
using namespace XmlRpc;

Node* ros::g_node = NULL;
bool ros::Node::s_initialized_ = false;
ros::Node::V_string ros::Node::s_args_;
ros::VP_string ros::Node::s_remappings_;
const  double MutexedXmlRpcClient::zombie_time = 30.0; // reap after 30 seconds

namespace ros
{
ROSOutAppenderPtr g_rosout_appender;
log4cxx::RollingFileAppenderPtr g_file_appender;
}

namespace ros_rpc
{


XmlRpcValue responseStr(int code, string msg, string response)
{
  XmlRpcValue v;
  v[0] = code;
  v[1] = msg;
  v[2] = response;
  return v;
}

XmlRpcValue responseInt(int code, string msg, int response)
{
  XmlRpcValue v;
  v[0] = int(code);
  v[1] = msg;
  v[2] = response;
  return v;
}

XmlRpcValue responseBool(int code, string msg, bool response)
{
  XmlRpcValue v;
  v[0] = int(code);
  v[1] = msg;
  v[2] = XmlRpcValue(response);
  return v;
}

class Shutdown : public XmlRpcServerMethod
{
public:
  Shutdown(XmlRpcServer *s, ros::Node* node)
  : XmlRpcServerMethod("shutdown", s)
  , node_(node)
  { }

  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
      num_params = params.size();
    if (num_params > 1 && string(params[1]).length())
    {
      node_->setShutdownMessage(string(params[1]));
      node_->shutdown();
    }
    else
    {
      _server->exit(); // only do this on the locally-generated shutdown()
    }
    result = responseInt(1, "", 0);
  }

private:
  ros::Node* node_;
};

class GetPid : public XmlRpcServerMethod
{
public:
  GetPid(XmlRpcServer *s, ros::Node* node)
  : XmlRpcServerMethod("getPid", s)
  , node_(node)
  { }

  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    result = responseInt(1, "", (int)getpid());
  }

private:
  ros::Node* node_;
};

class PublisherUpdate : public XmlRpcServerMethod
{
public:
  PublisherUpdate(XmlRpcServer *s, ros::Node* node)
  : XmlRpcServerMethod("publisherUpdate", s)
  , node_(node)
  { }

  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    vector<string> pubs;
    for (int idx = 0; idx < params[2].size(); idx++)
    {
      pubs.push_back(params[2][idx]);
    }
    if (node_->pubUpdate(params[1], pubs))
    {
      result = responseInt(1, "", 0);
    }
    else
    {
      std::string last_error = "Unknown Error";
      if ( g_rosout_appender != 0 )
      {
        last_error = g_rosout_appender->getLastError();
      }

      result = responseInt(0, last_error, 0);
    }
  }

private:
  ros::Node* node_;
};

class RequestTopic : public XmlRpcServerMethod
{
public:
  RequestTopic(XmlRpcServer *s, ros::Node* node)
  : XmlRpcServerMethod("requestTopic", s)
  , node_(node)
  { }

  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    if (!node_->requestTopic(params[1], params[2], result))
    {
      std::string last_error = "Unknown Error";
      if ( g_rosout_appender != 0 )
      {
        last_error = g_rosout_appender->getLastError();
      }

      result = responseInt(0, last_error, 0);
    }
  }

private:
  ros::Node* node_;
};

class GetBusStats : public XmlRpcServerMethod
{
public:
  GetBusStats(XmlRpcServer *s, ros::Node* node)
  : XmlRpcServerMethod("getBusStats", s)
  , node_(node)
  { }

  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    result[0] = 1;
    result[1] = string("");
    XmlRpcValue response;
    node_->getBusStats(result);
    result[2] = response;
  }

private:
  ros::Node* node_;
};

class GetBusInfo : public XmlRpcServerMethod
{
public:
  GetBusInfo(XmlRpcServer *s, ros::Node* node)
  : XmlRpcServerMethod("getBusInfo", s)
  , node_(node)
  { }

  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    result[0] = 1;
    result[1] = string("");
    XmlRpcValue response;
    node_->getBusInfo(response);
    result[2] = response;
  }

private:
  ros::Node* node_;
};

class ParamUpdate : public XmlRpcServerMethod
{
public:
  ParamUpdate(XmlRpcServer *s, ros::Node* node)
  : XmlRpcServerMethod("paramUpdate", s)
  , node_(node)
  { }

  void execute(XmlRpcValue &params, XmlRpcValue &result)
  {
    result[0] = 1;
    result[1] = string("");
    result[2] = 0;

    node_->paramUpdate((std::string)params[1], params[2]);
  }

private:
  ros::Node* node_;
};

}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

Node::Node(string _name, uint32_t options, int32_t master_retry_timeout) :
  name_(_name),
  shutting_down_(false),
  needs_shutdown_(false),
  ok_(true),
  master_retry_timeout_(master_retry_timeout),
  xmlrpc_port_(0),
  connection_id_counter_(0),
  namespace_("/")

{
  // This has to happen before we add any appenders to the system
  ROSCONSOLE_AUTOINIT;

  g_node = this;
  if (!s_initialized_)
  {
    ROS_ERROR("woah there! you didn't call ros::init(argc, argv) before\n" \
           "you tried to call a node-derived constructor!\n");
    ROS_BREAK();
  }
  char *ns_env = getenv("ROS_NAMESPACE");
  if (ns_env)
    namespace_ = ns_env;
  bool name_overridden = false;

  std::string log_file_name;

  VP_string::iterator remap_it = s_remappings_.begin();
  VP_string::iterator remap_end = s_remappings_.end();
  for (;remap_it != remap_end; ++remap_it)
  {
    std::string local_name = remap_it->first;
    std::string external_name = remap_it->second;

    if (local_name == "__log")
    {
      log_file_name = external_name;
    }
    else if (local_name == "__master")
    {
      if (!Node::splitURI(external_name, master_host_, master_port_))
      {
        ROS_ERROR( "Couldn't parse the __master argument [%s] into a host:port.", external_name.c_str());

        master_host_.clear();
        master_port_ = 0;
      }
    }
    else if (local_name == "__ip" || local_name == "__hostname")
    {
      ip_ = external_name;
    }
    else
    {
      if (external_name[0] == '~')
      {
        string tmp_name = joinPath(namespace_, name_); //namespace hasn't been joined to name_ yet, so do that here.
        external_name = joinPath(tmp_name, external_name.substr(1));
      }

      if (external_name[0] != '/')
      {
        external_name = joinPath(namespace_, external_name);
      }

      if (local_name == "__name")
      {
        name_overridden = true;
        name_ = external_name; // override name_, if present
      }
      else if (local_name == "__ns")
      {
        namespace_ = external_name; // override environment variable if present
      }
      else
      {
        name_cache_[local_name] = external_name;
      }
    }
  }

  if (!name_overridden)
    name_ = joinPath(namespace_, name_);

  if (name_[0] != '/')
    name_ = string("/") + name_; // names are globally referenced

  if (options & ANONYMOUS_NAME)
  {
    char buf[200];
    snprintf(buf, sizeof(buf), "_%llu", (unsigned long long)Time::now().toNSec());
    name_ += string(buf);
  }

  {
    // Log filename can be specified on the command line through __log
    // If it's been set, don't create our own name
    if (log_file_name.empty())
    {
      // Setup the logfile appender
      // Can't do this in rosconsole because the node name is not known
      pid_t pid = getpid();
      char* ros_log_env = getenv("ROS_LOG_DIR");

      if (ros_log_env)
      {
        log_file_name = ros_log_env + string("/");
      }
      else
      {
        ros_log_env = getenv("ROS_ROOT");

        if (ros_log_env)
        {
          log_file_name = ros_log_env + string("/log/");
        }
      }

      // sanitize the node name_ and tack it to the filename
      for (size_t i = 1; i < name_.length(); i++)
      {
        if (!isalnum(name_[i]))
        {
          log_file_name += '_';
        }
        else
        {
          log_file_name += name_[i];
        }
      }

      char pid_str[100];
      snprintf(pid_str, sizeof(pid_str), "%d", pid);
      log_file_name += string("_") + string(pid_str) + string(".log");
    }

    ROS_INFO("Logging to [%s]", log_file_name.c_str());

    const log4cxx::LoggerPtr& logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
    log4cxx::LayoutPtr layout = new log4cxx::PatternLayout("[%c] [%d] [thread %t]: [%p] %m\n");
    g_file_appender = new log4cxx::RollingFileAppender(layout, log_file_name, false);
    g_file_appender->setMaximumFileSize(100*1024*1024);
    g_file_appender->setMaxBackupIndex(10);
    log4cxx::helpers::Pool pool;
    g_file_appender->activateOptions(pool);
    logger->addAppender(g_file_appender);
  }

  if (master_host_.empty())
  {
    char *master_uri_env = getenv("ROS_MASTER_URI");
    if (!master_uri_env)
    {
      ROS_FATAL( "ROS_MASTER_URI is not defined in the environment. Either " \
                 "type the following or (preferrably) add this to your " \
                 "~/.bashrc file in order set up your " \
                 "local machine as a ROS master:\n\n" \
                 "export ROS_MASTER_URI=http://localhost:11311\n\n" \
                 "then, type 'roscore' in another shell to actually launch " \
                 "the master program.");
      ROS_BREAK();
    }
    // skip over http:// part if it's there
    if (!Node::splitURI(master_uri_env, master_host_, master_port_))
    {
      ROS_FATAL( "Couldn't parse the ROS_MASTER_URI [%s] into a host:port.",
          master_uri_env);
      ROS_BREAK();
    }
  }

  ROS_INFO("Starting node [%s]", name_.c_str());

  use_server_thread_ = false;
  if (!(options & DONT_START_SERVER_THREAD))
  {
    use_server_thread_ = true;
  }

  tcprosServerInit();

  if (use_server_thread_)
  {
    tcpros_server_thread_ = boost::thread(boost::bind(&Node::tcprosServerThreadFunc, this));
  }

  if (ip_.empty())
  {
    ip_ = determineIP();
  }

  ROS_INFO("Binding on hostname/ip [%s]", ip_.c_str());

  char port_cstr[50];
  //assert(xmlrpc_port != 0);
  snprintf(port_cstr, sizeof(port_cstr), "%d", xmlrpc_port_);
  xmlrpc_uri_ = string("http://") + ip_ + string(":") +
               string(port_cstr) + string("/");
  ROS_DEBUG( "xmlrpc URI = %s", xmlrpc_uri_.c_str());
  if (!(options & DONT_HANDLE_SIGINT))
  {
    setupSigintHandler();
  }

  // Add our rosout appender
  if (!(options & DONT_ADD_ROSOUT_APPENDER))
  {
    ROS_ASSERT(!g_rosout_appender);
    g_rosout_appender = new ROSOutAppender(this);
    const log4cxx::LoggerPtr& logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
    logger->addAppender(g_rosout_appender);
  }

  //Set up the timing callback
  bool use_sim_time = false;
  param("/use_sim_time", use_sim_time, use_sim_time);
  if (use_sim_time)
  {
    Time::setNow(ros::Time());
  }
  subscribe("/time", time_msg_, &Node::timeCallback, 1);
}

Node::~Node()
{
  if (g_rosout_appender != 0)
  {
    const log4cxx::LoggerPtr& logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
    logger->removeAppender(g_rosout_appender);
    g_rosout_appender = log4cxx::AppenderPtr();
  }

  ROS_DEBUG( "entering ros::Node destructor");
  shutdown(); // if we haven't disconnected already, do so

  // Join the TCPROS server thread
  if(use_server_thread_)
  {
    tcpros_server_thread_.join();
  }

  // kill the last few clients that were started in the shutdown process
  for (vector<MutexedXmlRpcClient>::iterator i = xmlrpc_clients_.begin();
       i != xmlrpc_clients_.end(); ++i)
  {
    for (int wait_count = 0; i->in_use && wait_count < 10; wait_count++)
    {
      ROS_INFO("waiting for xmlrpc connection to finish...");
      usleep(10000);
    }
    // time's up. UR DONE.
    i->c->close();
    delete i->c;
  }
  xmlrpc_clients_.clear();

  if (g_file_appender != 0)
  {
    const log4cxx::LoggerPtr& logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
    logger->removeAppender(g_file_appender);
    g_file_appender = log4cxx::RollingFileAppenderPtr();
  }

  // Set the instance pointer to NULL, to allow applications that create
  // and destroy the node multiple times in one run.
  g_node = NULL;
}

Node *ros::Node::instance()
{
  return g_node;
}

void Node::requestShutdown()
{
  needs_shutdown_ = true;
}

std::string Node::cleanName(const std::string& name)
{
  std::string clean = name;

  size_t pos = clean.find("//");
  while (pos != std::string::npos)
  {
    clean.erase(pos, 1);
    pos = clean.find("//", pos);
  }

  if (*clean.rbegin() == '/')
  {
    clean.erase(clean.size() - 1, 1);
  }

  return clean;
}

std::string Node::mapName(const std::string& name)
{
  M_string::iterator it = name_cache_.find(name);
  if (it != name_cache_.end())
  {
    return it->second;
  }

  std::string mapped = name;

  if (mapped[0] == '~')
  {
    mapped = joinPath(getName(), mapped.substr(1));
  }

  if (mapped[0] != '/')
  {
    mapped = joinPath(namespace_, mapped);
  }

  if (mapped[0] != '/') // make it global if join didn't
  {
    mapped.insert(0, "/");
  }

  mapped = cleanName(mapped);

  name_cache_.insert(std::make_pair(name, mapped));

  return mapped;
}

void Node::getAdvertisedTopics(std::vector<std::string>& topics)
{
  boost::mutex::scoped_lock lock(advertised_topic_names_mutex_);

  topics.resize(advertised_topic_names_.size());
  std::copy(advertised_topic_names_.begin(),
            advertised_topic_names_.end(),
            topics.begin());
}

PublicationPtr Node::getTopic(std::string topic)
{
  PublicationPtr ret;
  boost::mutex::scoped_lock lock(advertised_topics_mutex_);

  std::string topic_remapped = mapName(topic);
  for (V_Publication::iterator t = advertised_topics_.begin();
       t != advertised_topics_.end(); ++t)
  {
    if(!(*t)->isDropped() && ((*t)->getName() == topic_remapped))
    {
      ret = *t;
      break;
    }
  }

  return ret;
}

bool Node::addSubCallback(const std::string &_topic,
                             Message* m, AbstractFunctor *fp, int max_queue)
{
  // spin through the subscriptions and see if we find a match. if so, use it.
  bool found = false;

  SubscriptionPtr sub;

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    if (shuttingDown())
    {
      return false;
    }

    for (L_Subscription::iterator s = subscriptions_.begin();
         s != subscriptions_.end() && !found; ++s)
    {
      sub = *s;
      if (!sub->isDropped() && sub->getName() == _topic && sub->md5sum() == m->__getMD5Sum())
      {
        found = true;
        break;
      }
    }
  }

  if (found)
  {
    if (sub->addFunctorMessagePair(fp, m))
    {
      if ((sub->getMaxQueue() != 0 && sub->getMaxQueue() < max_queue) || (max_queue == 0 && sub->getMaxQueue() != 0))
      {
        ROS_WARN("Changing subscription '%s' max_queue size from %d to %d\n", sub->getName().c_str(), sub->getMaxQueue(), max_queue);

        sub->setMaxQueue(max_queue);
      }
    }
    else
    {
      return false;
    }
  }

  return found;
}

// this function has the subscription code that doesn't need to be templated.
bool Node::subscribe(const std::string &name, Message* m, AbstractFunctor *cb,
                     bool threaded, int max_queue, const std::string &datatype)
{
  std::string mapped_name = mapName(name);
  if (addSubCallback(mapped_name, m, cb, max_queue))
  {
    return true;
  }

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    if (shuttingDown())
    {
      delete cb;
      return false;
    }
  }

  SubscriptionPtr s(new Subscription(mapped_name, m, cb, threaded, max_queue));
  
  if (!registerSubscriber(s, datatype))
  {
    ROS_WARN("couldn't register subscriber on topic [%s]", name.c_str());
    s->shutdown();
    return false;
  }

  {
    boost::mutex::scoped_lock lock(subs_mutex_);
    subscriptions_.push_back(s);
  }

  return true;
}

void Node::setMasterRetryTimeout(int32_t milliseconds)
{
  master_retry_timeout_ = milliseconds;
  ROS_INFO("Timeout set at %d ms", master_retry_timeout_);
}

bool Node::executeMasterXmlrpc(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload, bool wait_for_master)
{
  boost::system_time start_time = boost::get_system_time();

  XmlRpcClient *c = getXMLRPCClient(master_host_.c_str(), master_port_, "/");
  bool printed = false;
  bool slept = false;
  do
  {
    bool b = false;

    {
#if defined(__APPLE__)
      boost::mutex::scoped_lock lock(xmlrpc_call_mutex_);
#endif

      b = c->execute(method.c_str(), request, response);
    }

    if (!b && ok())
    {
      if (!printed)
      {
        ROS_ERROR("[%s] Failed to contact master at [%s:%d].  %s", method.c_str(), master_host_.c_str(), master_port_, wait_for_master ? "Retrying..." : "");
        printed = true;
      }

      if (!wait_for_master)
      {
        releaseXMLRPCClient(c);
        return false;
      }

      if (master_retry_timeout_ >= 0)
      {
        boost::system_time current_time = boost::get_system_time();

        if ((current_time - start_time) >= boost::posix_time::milliseconds(master_retry_timeout_))
        {
          ROS_ERROR("[%s] Timed out trying to connect to the master after [%d] milliseconds", method.c_str(), master_retry_timeout_);
          releaseXMLRPCClient(c);
          return false;
        }
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(50));
      slept = true;
    }
    else
    {
      if (!validateXmlrpcResponse(method, response, payload) && ok())
      {
        releaseXMLRPCClient(c);

        return false;
      }

      break;
    }
  } while(ok());

  if (ok() && slept)
  {
    ROS_INFO("Connected to master at [%s:%d]", master_host_.c_str(), master_port_);
  }

  releaseXMLRPCClient(c);

  return true;
}

// this function handles all the stuff that doesn't need to be templated
bool Node::advertise(const string& _topic, const string& original_topic_name,
                      const string& datatype,
                      const string& md5sum,
                      const SubscriptionConnectionCallback& connect_cb,
                      const SubscriptionConnectionCallback& disconnect_cb,
                      size_t max_queue)
{
  std::string topic = mapName(_topic);

  if (datatype == "*")
  {
    ROS_WARN("Advertising on topic [%s] with datatype [*].  If you are not playing back an old bag file, this is a problem.", _topic.c_str());
  }

  if (md5sum == "*")
  {
    ROS_WARN("Advertising on topic [%s] with md5sum [*].  If you are not playing back an old bag file, this is a problem.", _topic.c_str());
  }

  {
    boost::mutex::scoped_lock lock(advertised_topics_mutex_);

    if (shuttingDown())
    {
      return false;
    }

    if (isTopicAdvertised(topic))
    {
      return false;
    }

    advertised_topics_.push_back(
        PublicationPtr(new Publication(topic, original_topic_name, datatype, md5sum,
                        connect_cb, disconnect_cb, max_queue)));
  }


  {
    boost::mutex::scoped_lock lock(advertised_topic_names_mutex_);
    advertised_topic_names_.push_back(topic);
  }

  // Check whether we've already subscribed to this topic.  If so, we'll do
  // the self-subscription here, to avoid the deadlock that would happen if
  // the ROS thread later got the publisherUpdate with its own XMLRPC URI.
  // The assumption is that advertise() is called from somewhere other
  // than the ROS thread.
  bool found = false;
  SubscriptionPtr sub;
  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    for (L_Subscription::iterator s = subscriptions_.begin();
         s != subscriptions_.end() && !found; ++s)
    {
      if ((*s)->getName() == topic && (*s)->md5sum() == md5sum && !(*s)->isDropped())
      {
        found = true;
        sub = *s;
        break;
      }
    }
  }

  if(found)
  {
    sub->negotiateConnection(xmlrpc_uri_);
  }

  XmlRpcValue args, result, payload;
  args[0] = name_;
  args[1] = topic;
  args[2] = datatype;
  args[3] = xmlrpc_uri_;
  executeMasterXmlrpc("registerPublisher", args, result, payload, true);

  return true;
}

bool Node::unadvertise(const std::string &topic)
{
  PublicationPtr pub;

  {
    boost::mutex::scoped_lock lock(advertised_topics_mutex_);

    if (shuttingDown())
    {
      return false;
    }

    std::string topic_remapped = mapName(topic);
    for (V_Publication::iterator i = advertised_topics_.begin();
         i != advertised_topics_.end(); ++i)
    {
      if(((*i)->getName() == topic_remapped) && (!(*i)->isDropped()))
      {
        pub = *i;
        advertised_topics_.erase(i);
        break;
      }
    }
  }

  if (pub)
  {
    unregisterPublisher(pub->getName());
    pub->drop();

    {
      boost::mutex::scoped_lock lock(advertised_topic_names_mutex_);
      advertised_topic_names_.remove(pub->getName());
    }

    return true;
  }

  return false;
}

bool Node::unregisterPublisher(const std::string& topic)
{
  XmlRpcValue args, result, payload;
  args[0] = name_;
  args[1] = topic;
  args[2] = xmlrpc_uri_;
  executeMasterXmlrpc("unregisterPublisher", args, result, payload, false);

  return true;
}

bool Node::advertiseService(const std::string &serv_name, const std::string& md5sum, const std::string& request_data_type,
                              const std::string& response_data_type, int thread_pool_size, AbstractServiceFunctor *cb)
{
  {
    boost::mutex::scoped_lock lock(service_servers_mutex_);

    if (shuttingDown())
    {
      return false;
    }

    if (isServiceAdvertised(serv_name))
    {
      ROS_ERROR("Tried to advertise a service that is already advertised in this node [%s]", serv_name.c_str());
      delete cb;
      return false;
    }

    ServiceServerPtr ss(new ServiceServer(serv_name, md5sum, request_data_type, response_data_type, cb, thread_pool_size));
    service_servers_.push_back(ss);
  }

  XmlRpcValue args, result, payload;
  args[0] = name_;
  args[1] = serv_name;
  char uri_buf[1024];
  snprintf(uri_buf, sizeof(uri_buf), "rosrpc://%s:%d",
           ip_.c_str(), tcpserver_transport_->getServerPort());
  ROS_DEBUG( "registering service [%s] with uri [%s]", serv_name.c_str(), uri_buf);
  args[2] = string(uri_buf);
  args[3] = xmlrpc_uri_;
  executeMasterXmlrpc("registerService", args, result, payload, true);

  return true;
}

bool Node::unadvertiseService(const string &_serv_name)
{
  std::string serv_name;
  ServiceServerPtr ss;
  {
    boost::mutex::scoped_lock lock(service_servers_mutex_);

    if (shuttingDown())
    {
      return false;
    }

    serv_name = mapName(_serv_name);

    for (L_ServiceServer::iterator i = service_servers_.begin();
         i != service_servers_.end(); ++i)
    {
      if((*i)->getName() == serv_name && !(*i)->isDropped())
      {
        ss = *i;
        service_servers_.erase(i);
        break;
      }
    }
  }

  if (ss)
  {
    unregisterService(ss->getName());
    ROS_DEBUG( "shutting down service [%s]", ss->getName().c_str());
    ss->drop();
    return true;
  }

  return false;
}

bool Node::unregisterService(const std::string& service)
{
  XmlRpcValue args, result, payload;
  args[0] = name_;
  args[1] = service;
  char uri_buf[1024];
  snprintf(uri_buf, sizeof(uri_buf), "rosrpc://%s:%d",
           ip_.c_str(), tcpserver_transport_->getServerPort());
  args[2] = string(uri_buf);

  executeMasterXmlrpc("unregisterService", args, result, payload, false);

  return true;
}

bool Node::advertise(const string &_topic,
                     const Message& msgref, size_t max_queue)
{
  return advertise(_topic, _topic, msgref.__getDataType(),
                    msgref.__getMD5Sum(), NULL, NULL, max_queue);
}

bool Node::isTopicAdvertised(const string &topic)
{
  bool found = false;
  for (V_Publication::iterator t = advertised_topics_.begin();
       !found && t != advertised_topics_.end(); ++t)
  {
    if (((*t)->getName() == topic) && (!(*t)->isDropped()))
    {
      found = true;
      break;
    }
  }

  return found;
}

bool Node::isServiceAdvertised(const string &serv_name)
{
  bool found = false;
  for (L_ServiceServer::iterator s = service_servers_.begin();
       !found && s != service_servers_.end(); ++s)
  {
    if (((*s)->getName() == serv_name) && !(*s)->isDropped())
    {
      found = true;
      break;
    }
  }

  return found;
}

static bool isPrivateIP(const char *ip)
{
  bool b = !strncmp("192.168", ip, 7) || !strncmp("10.", ip, 3) ||
           !strncmp("169.254", ip, 7);
  return b;
}

string Node::determineIP()
{
  // First, did the user set ROS_HOSTNAME?
  char *ip_env = getenv("ROS_HOSTNAME");
  if (ip_env)
  {
    ROS_DEBUG( "determineIP: using value of ROS_HOSTNAME:%s:", ip_env);
    return string(ip_env);
  }

  // Second, did the user set ROS_IP?
  ip_env = getenv("ROS_IP");
  if (ip_env)
  {
    ROS_DEBUG( "determineIP: using value of ROS_IP:%s:", ip_env);
    return string(ip_env);
  }

  // Third, try the hostname
  char host[1024];
  memset(host,0,sizeof(host));
  if(gethostname(host,sizeof(host)-1) != 0)
  {
    ROS_ERROR("determineIP: gethostname failed");
  }
  // We don't want localhost to be our ip
  else if(strlen(host) && strcmp("localhost", host))
  {
    return string(host);
  }

  // Fourth, fall back on interface search, which will yield an IP address

#ifdef HAVE_IFADDRS_H
  struct ifaddrs *ifa = NULL, *ifp = NULL;
  int rc;
  if ((rc = getifaddrs(&ifp)) < 0)
  {
    ROS_FATAL("error in getifaddrs: [%s]", strerror(rc));
    ROS_BREAK();
  }
  char preferred_ip[200] = {0};
  for (ifa = ifp; ifa; ifa = ifa->ifa_next)
  {
    char ip_[200];
    socklen_t salen;
    if (!ifa->ifa_addr)
      continue; // evidently this interface has no ip address
    if (ifa->ifa_addr->sa_family == AF_INET)
      salen = sizeof(struct sockaddr_in);
    else if (ifa->ifa_addr->sa_family == AF_INET6)
      salen = sizeof(struct sockaddr_in6);
    else
      continue;
    if (getnameinfo(ifa->ifa_addr, salen, ip_, sizeof(ip_), NULL, 0,
                    NI_NUMERICHOST) < 0)
    {
      ROS_DEBUG( "getnameinfo couldn't get the ip of interface [%s]", ifa->ifa_name);
      continue;
    }
    //ROS_INFO( "ip of interface [%s] is [%s]", ifa->ifa_name, ip);
    // prefer non-private IPs over private IPs
    if (!strcmp("127.0.0.1", ip_) || strchr(ip_,':'))
      continue; // ignore loopback unless we have no other choice
    if (ifa->ifa_addr->sa_family == AF_INET6 && !preferred_ip[0])
      strcpy(preferred_ip, ip_);
    else if (isPrivateIP(ip_) && !preferred_ip[0])
      strcpy(preferred_ip, ip_);
    else if (!isPrivateIP(ip_) &&
             (isPrivateIP(preferred_ip) || !preferred_ip[0]))
      strcpy(preferred_ip, ip_);
  }
  freeifaddrs(ifp);
  if (!preferred_ip[0])
  {
    ROS_ERROR( "Couldn't find a preferred IP via the getifaddrs() call; I'm assuming that your IP "
        "address is 127.0.0.1.  This should work for local processes, "
        "but will almost certainly not work if you have remote processes."
        "Report to the ROS development team to seek a fix.");
    return string("127.0.0.1");
  }
  ROS_DEBUG( "preferred IP is guessed to be %s", preferred_ip);
  return string(preferred_ip);
#else
  // @todo Fix IP determination in the case where getifaddrs() isn't
  // available.
  ROS_ERROR( "You don't have the getifaddrs() call; I'm assuming that your IP "
             "address is 127.0.0.1.  This should work for local processes, "
             "but will almost certainly not work if you have remote processes."
             "Report to the ROS development team to seek a fix.");
  return string("127.0.0.1");
#endif
}

void ros::basicSigintHandler(int sig)
{
  g_node->requestShutdown();
}

void Node::setupSigintHandler()
{
  signal(SIGINT, ros::basicSigintHandler);
}

void Node::spin()
{
  while (ok_)
  {
    if(use_server_thread_)
    {
      usleep(10000);
    }
    else
    {
      if (!shuttingDown())
      {
        tcprosServerUpdate();
      }
    }
  }
}

bool Node::registerSubscriber(const SubscriptionPtr& s, const string &datatype)
{
  XmlRpcValue args, result, payload;
  args[0] = name_;
  args[1] = s->getName();
  args[2] = datatype;
  args[3] = xmlrpc_uri_;

  if (!executeMasterXmlrpc("registerSubscriber", args, result, payload, true))
  {
    return false;
  }

  vector<string> pub_uris;
  bool self_subscribed = false;
  for (int i = 0; i < payload.size(); i++)
  {
    if (payload[i] == xmlrpc_uri_)
    {
      self_subscribed = true;
    }
    else
    {
      pub_uris.push_back(string(payload[i]));
    }
  }

  s->pubUpdate(pub_uris);

  if (self_subscribed)
  {
    s->negotiateConnection(xmlrpc_uri_);
  }

  return true;
}

bool Node::unregisterSubscriber(const string &topic)
{
  XmlRpcValue args, result, payload;
  args[0] = name_;
  args[1] = topic;
  args[2] = xmlrpc_uri_;

  executeMasterXmlrpc("unregisterSubscriber", args, result, payload, false);

  return true;
}

bool Node::getPublishedTopics(vector<pair<string, string> > *topics)
{
  XmlRpcValue args, result, payload;
  args[0] = name_;
  args[1] = ""; //TODO: Fix this

  if (!executeMasterXmlrpc("getPublishedTopics", args, result, payload, true))
  {
    return false;
  }

  topics->clear();
  for (int i = 0; i < payload.size(); i++)
  {
    topics->push_back(pair<string, string>(string(payload[i][0]), string(payload[i][1])));
  }

  return true;
}

bool Node::validateXmlrpcResponse(string method, XmlRpcValue &response,
                                    XmlRpcValue &payload)
{
  if (response.getType() != XmlRpcValue::TypeArray)
  {
    ROS_DEBUG("XML-RPC call [%s] didn't return an array",
        method.c_str());
    return false;
  }
  if (response.size() != 3)
  {
    ROS_DEBUG("XML-RPC call [%s] didn't return a 3-element array",
        method.c_str());
    return false;
  }
  if (response[0].getType() != XmlRpcValue::TypeInt)
  {
    ROS_DEBUG("XML-RPC call [%s] didn't return a int as the 1st element",
        method.c_str());
    return false;
  }
  int status_code = response[0];
  if (response[1].getType() != XmlRpcValue::TypeString)
  {
    ROS_DEBUG("XML-RPC call [%s] didn't return a string as the 2nd element",
        method.c_str());
    return false;
  }
  string status_string = response[1];
  if (status_code != 1)
  {
    ROS_DEBUG("XML-RPC call [%s] returned an error (%d): [%s]",
        method.c_str(), status_code, status_string.c_str());
    return false;
  }
  payload = response[2];
  return true;
}

bool Node::pubUpdate(const string &topic, const vector<string> &pubs)
{
  SubscriptionPtr sub;
  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    if (shuttingDown())
    {
      return false;
    }

    ROS_DEBUG("Received update for topic [%s] (%d publishers)", topic.c_str(), (int)pubs.size());
    // find the subscription
    for (L_Subscription::const_iterator s  = subscriptions_.begin();
                                            s != subscriptions_.end(); ++s)
    {
      if ((*s)->getName() != topic || (*s)->isDropped())
        continue;

      sub = *s;
      break;
    }

  }

  if (sub)
  {
    sub->pubUpdate(pubs);
  }
  else
  {
    ROS_DEBUG("got a request for updating publishers of topic %s, but I " \
              "don't have any subscribers to that topic.", topic.c_str());
  }

  return false;
}

bool Node::requestTopic(const string &topic,
                         XmlRpcValue &protos,
                         XmlRpcValue &ret)
{
  for (int proto_idx = 0; proto_idx < protos.size(); proto_idx++)
  {
    XmlRpcValue proto = protos[proto_idx]; // save typing
    if (proto.getType() != XmlRpcValue::TypeArray)
    {
      ROS_ERROR( "requestTopic protocol list was not a list of lists");
      return false;
    }

    if (proto[0].getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR( "requestTopic received a protocol list in which a sublist " \
                 "did not start with a string");
      return false;
    }

    string proto_name = proto[0];
    if (proto_name == string("TCPROS"))
    {
      // when we get more complicated and have more than one protocol,
      // this logic will be a bit more sophisticated.
      XmlRpcValue tcpros_params;
      tcpros_params[0] = string("TCPROS");
      tcpros_params[1] = ip_;
      tcpros_params[2] = tcpserver_transport_->getServerPort();
      ret[0] = int(1);
      ret[1] = string();
      ret[2] = tcpros_params;
      return true;
    }
    else
    {
      ROS_DEBUG( "an unsupported protocol was offered: [%s]",
          proto_name.c_str());
    }
  }

  ROS_ERROR( "Currently, roscpp only supports TCPROS. The caller to " \
             "requestTopic did not support TCPROS, so there are no " \
             "protocols in common.");
  return false;
}

unsigned int Node::getNewConnectionID()
{
  boost::mutex::scoped_lock lock(connection_id_counter_mutex_);
  unsigned int id = connection_id_counter_++;
  return id;
}

void Node::tcprosServerInit()
{
  // We don't want to catch SIGPIPE in this thread
  signal(SIGPIPE, SIG_IGN);

  ///////////////////////
  // XmlRpc server stuff
  shutdown_xmlrpc_object_ = new ros_rpc::Shutdown(&xmlrpc_server_, this);
  getpid_xmlrpc_object_ = new ros_rpc::GetPid(&xmlrpc_server_, this);
  publisher_update_xmlrpc_object_ = new ros_rpc::PublisherUpdate(&xmlrpc_server_, this);
  request_topic_xmlrpc_object_ = new ros_rpc::RequestTopic(&xmlrpc_server_, this);
  get_bus_stats_xmlrpc_object_ = new ros_rpc::GetBusStats(&xmlrpc_server_, this);
  get_bus_info_xmlrpc_object_ = new ros_rpc::GetBusInfo(&xmlrpc_server_, this);
  param_update_xmlrpc_object_ = new ros_rpc::ParamUpdate(&xmlrpc_server_, this);

  bool bound = xmlrpc_server_.bindAndListen(0);
  ROS_ASSERT(bound);
  xmlrpc_port_ = xmlrpc_server_.get_port();
  ROS_ASSERT(xmlrpc_port_ != 0);
  ROS_INFO( "Started XML-RPC server on port [%d]", xmlrpc_port_);
  ///////////////////////

  // Bring up the TCP listener socket
  tcpserver_transport_ = TransportTCPPtr(new TransportTCP(&poll_set_));
  if (!tcpserver_transport_->listen(0, MAX_TCPROS_CONN_QUEUE, boost::bind(&Node::tcprosAcceptConnection, this, _1)))
  {
    ROS_FATAL("Listen failed");
    ROS_BREAK();
  }

  ROS_INFO( "Started TCPROS server on port [%d]", tcpserver_transport_->getServerPort());
}

void Node::tcprosServerFini()
{
  tcpserver_transport_->close();

  delete shutdown_xmlrpc_object_;
  delete getpid_xmlrpc_object_;
  delete publisher_update_xmlrpc_object_;
  delete request_topic_xmlrpc_object_;
  delete get_bus_info_xmlrpc_object_;
  delete get_bus_stats_xmlrpc_object_;
  delete param_update_xmlrpc_object_;

  ROS_DEBUG( "tearing down connections");
  for(S_Connection::iterator itr = connections_.begin();
      itr != connections_.end();
      itr++)
  {
    const ConnectionPtr& conn = *itr;
    conn->drop();
  }

  connections_.clear();
}

bool Node::onConnectionHeaderReceived(const ConnectionPtr& conn, const Header& header)
{
  bool ret = false;
  std::string val;
  if (header.getValue("topic", val))
  {
    ROS_DEBUG("Connection: Creating Publisher for topic [%s] connected to [%s]", val.c_str(), conn->getTransport()->getTransportInfo().c_str());

    PublisherPtr pub(new Publisher());
    pub->initialize(conn);
    ret = pub->handleHeader(header);
  }
  else if (header.getValue("service", val))
  {
    ROS_DEBUG("Connection: Creating ServiceLink for service [%s] connected to [%s]", val.c_str(), conn->getTransport()->getTransportInfo().c_str());

    ServiceLinkPtr link(new ServiceLink());
    link->initialize(conn);
    ret = link->handleHeader(header);
  }
  else
  {
    ROS_ERROR("Got a connection for a type other than 'topic' or 'service'.  Fail.");
    return false;
  }

  return ret;
}

void Node::onConnectionDropped(const ConnectionPtr& conn)
{
  boost::mutex::scoped_lock lock(dropped_connections_mutex_);
  dropped_connections_.push_back(conn);
}

void Node::removeDroppedConnections()
{
  boost::mutex::scoped_lock dropped_lock(dropped_connections_mutex_);
  boost::mutex::scoped_lock conn_lock(connections_mutex_);

  V_Connection::iterator conn_it = dropped_connections_.begin();
  V_Connection::iterator conn_end = dropped_connections_.end();
  for (;conn_it != conn_end; ++conn_it)
  {
    const ConnectionPtr& conn = *conn_it;

    size_t prev_size = connections_.size();
    connections_.erase(conn);
    ROS_ASSERT(connections_.size() == (prev_size - 1));
  }

  dropped_connections_.clear();
}

void Node::tcprosServerUpdate()
{
  {
    static bool printed = false;
    static int count = 0;
    if (!printed)
    {
      if (count > 100)
      {
        if (typeid(*this) != typeid(ros::Node))
        {
          ROS_WARN("Deriving from ros::Node has been deprecated! (class %s)", typeid(*this).name());
        }

        printed = true;
      }

      ++count;
    }
  }

  if (needs_shutdown_)
  {
    shutdown();

    return;
  }

  // Update the XMLRPC server, non-blocking
  xmlrpc_server_.work(0.0);

  if (shutting_down_)
  {
    return;
  }

  removeDroppedConnections();

  poll_set_.update(10);
}

void Node::tcprosServerThreadFunc()
{
  disableAllSignalsInThisThread();

  while(!shutting_down_)
    tcprosServerUpdate();
}

void Node::publish(const std::string &_topic, const Message& m)
{
  std::string topic = mapName(_topic);
  {
    boost::mutex::scoped_lock lock(advertised_topics_mutex_);

    if (shuttingDown())
    {
      return;
    }

    if(!isTopicAdvertised(topic))
    {
      ROS_ERROR("attempted to publish to topic %s, which was not " \
          "previously advertised. call advertise(\"%s\") first.",
          topic.c_str(), topic.c_str());
      return;
    }
  }

  boost::mutex::scoped_lock lock(advertised_topics_mutex_);
  for (V_Publication::iterator t = advertised_topics_.begin();
       t != advertised_topics_.end(); ++t)
  {
    if ((*t)->getName() == topic)
    {
      if (m.__getDataType() != ((*t)->getDataType()))
      {
        ROS_ERROR("Topic [%s] advertised as [%s], but published as [%s]", topic.c_str(), (*t)->getDataType().c_str(), m.__getDataType().c_str());
      }
      else
      {
        publish(*t, m);
      }
      break;
    }
  }
}

void Node::publish(const PublicationPtr& p, const Message& m)
{
  p->incrementSequence();
  if (p->hasSubscribers())
  {
    uint32_t msg_len = m.serializationLength();
    boost::shared_array<uint8_t> buf = boost::shared_array<uint8_t>(new uint8_t[msg_len + 4]);

    *((uint32_t*)buf.get()) = msg_len;
    m.serialize(buf.get() + 4, p->getSequence());
    ROS_DEBUG_NAMED("superdebug", "Publishing message on topic [%s] with sequence number [%d] of length [%d]", p->getName().c_str(), p->getSequence(), msg_len);
    p->enqueueMessage(buf, msg_len + 4);
  }
}

void ros::init(int& _argc, char** _argv)
{
  Node::s_args_.clear();

  int full_argc = _argc;
  // now, move the remapping argv's to the end, and decrement argc as needed
  for (int i = 0; i < _argc; )
  {
    std::string arg = _argv[i];
    size_t pos = arg.find(":=");
    if (pos != std::string::npos)
    {
      string local_name = arg.substr(0, pos);
      string external_name = arg.substr(pos + 2);

      Node::s_remappings_.push_back(std::make_pair(local_name, external_name));
      Node::s_args_.push_back(_argv[i]);

      // shuffle everybody down and stuff this guy at the end of argv
      char *tmp = _argv[i];
      for (int j = i; j < full_argc - 1; j++)
        _argv[j] = _argv[j+1];
      _argv[_argc-1] = tmp;
      _argc--;
    }
    else
    {
      i++; // move on, since we didn't shuffle anybody here to replace it
    }
  }

  Node::s_initialized_ = true;
}

void ros::init(const VP_string& remappings)
{
  Node::s_remappings_ = remappings;

  Node::s_initialized_ = true;
}

void Node::setParam(const string &key, const XmlRpcValue &v)
{
  if (shuttingDown())
  {
    return;
  }

  std::string mapped_key = mapName(key);

  XmlRpcValue params, result, payload;
  params[0] = name_;
  params[1] = mapped_key;
  params[2] = v;

  {
    // Lock around the execute to the master in case we get a parameter update on this value between
    // executing on the master and setting the parameter in the params_ list.
    boost::mutex::scoped_lock lock(params_mutex_);

    if (executeMasterXmlrpc("setParam", params, result, payload, true))
    {
      // Update our cached params list now so that if getParam() is called immediately after setParam()
      // we already have the cached state and our value will be correct
      if (subscribed_params_.find(mapped_key) != subscribed_params_.end())
      {
        params_[mapped_key] = v;
      }
    }
  }
}

void Node::setParam(const string &key, const string &s)
{
  if (shuttingDown())
  {
    return;
  }

  // construct xmlrpc_c::value object of the string and
  // call setParam(key, xmlvalue);
  XmlRpcValue v(s);
  setParam(key, v);
}

void Node::setParam(const string &key, const char* s)
{
  if (shuttingDown())
  {
    return;
  }

  // construct xmlrpc_c::value object of the string and
  // call setParam(key, xmlvalue);
  string sxx = string(s);
  XmlRpcValue v(sxx);
  ROS_DEBUG("setParam(): set %s, type %d\n", key.c_str(), v.getType());
  setParam(key, v);
}

void Node::setParam(const string &key, double d)
{
  if (shuttingDown())
  {
    return;
  }

  XmlRpcValue v(d);
  setParam(key, v);
}

void Node::setParam(const string &key, int i)
{
  if (shuttingDown())
  {
    return;
  }

  XmlRpcValue v(i);
  setParam(key, v);
}

bool Node::hasParam(const string &key)
{
  if (shuttingDown())
  {
    return false;
  }

  XmlRpcValue params, result, payload;
  params[0] = name_;
  params[1] = mapName(key);
  //params[1] = key;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!executeMasterXmlrpc("hasParam", params, result, payload, false))
  {
    return false;
  }

  return payload;
}

bool Node::deleteParam(const string &key)
{
  if (shuttingDown())
  {
    return false;
  }

  std::string mapped_key = mapName(key);

  {
    boost::mutex::scoped_lock lock(params_mutex_);

    S_string::iterator sub_it = subscribed_params_.find(mapped_key);
    if (sub_it != subscribed_params_.end())
    {
      subscribed_params_.erase(sub_it);

      M_Param::iterator param_it = params_.find(mapped_key);
      if (param_it != params_.end())
      {
        params_.erase(param_it);
      }
    }
  }

  XmlRpcValue params, result, payload;
  params[0] = name_;
  params[1] = mapped_key;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!executeMasterXmlrpc("deleteParam", params, result, payload, false))
  {
    return false;
  }

  return true;
}

bool Node::getParam(const string &key, XmlRpcValue &v, bool use_cache)
{
  if (shuttingDown())
  {
    return false;
  }

  std::string mapped_key = mapName(key);

  if (use_cache)
  {
    boost::mutex::scoped_lock lock(params_mutex_);

    if (subscribed_params_.find(mapped_key) != subscribed_params_.end())
    {
      M_Param::iterator it = params_.find(mapped_key);
      if (it != params_.end())
      {
        ROS_DEBUG("Using cached parameter value for key [%s]", mapped_key.c_str());

        v = it->second;
        return true;
      }
    }
    else
    {
      // parameter we've never seen before, register for update from the master
      if (subscribed_params_.insert(mapped_key).second)
      {
        XmlRpcValue params, result, payload;
        params[0] = name_;
        params[1] = xmlrpc_uri_;
        params[2] = mapped_key;

        if (!executeMasterXmlrpc("subscribeParam", params, result, payload, false))
        {
          subscribed_params_.erase(mapped_key);
        }
        else
        {
          ROS_DEBUG("Subscribed to parameter [%s]", mapped_key.c_str());
        }
      }
    }
  }

  XmlRpcValue params, result;
  params[0] = name_;
  params[1] = mapped_key;

  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!executeMasterXmlrpc("getParam", params, result, v, false))
  {
    return false;
  }

  if (use_cache && v.valid())
  {
    boost::mutex::scoped_lock lock(params_mutex_);

    ROS_DEBUG("Caching parameter [%s] with value type [%d]", mapped_key.c_str(), v.getType());
    params_[mapped_key] = v;
  }

  return true;
}

bool Node::getParam(const string &key, string &s, bool use_cache)
{
  if (shuttingDown())
  {
    return false;
  }

  XmlRpcValue v;
  if (!getParam(key, v, use_cache))
    return false;
  if (v.getType() != XmlRpcValue::TypeString)
    return false;
  s = string(v);
  return true;
}

bool Node::getParam(const string &key, double &d, bool use_cache)
{
  if (shuttingDown())
  {
    return false;
  }

  XmlRpcValue v;
  if (!getParam(key, v, use_cache))
  {
    return false;
  }

  if (v.getType() == XmlRpcValue::TypeInt)
  {
    d = (int)v;
  }
  else if (v.getType() != XmlRpcValue::TypeDouble)
  {
    return false;
  }
  else
  {
    d = v;
  }

  return true;
}

bool Node::getParam(const string &key, int &i, bool use_cache)
{
  if (shuttingDown())
  {
    return false;
  }

  XmlRpcValue v;
  if (!getParam(key, v, use_cache))
  {
    return false;
  }

  if (v.getType() == XmlRpcValue::TypeDouble)
  {
    double d = v;

    if (fmod(d, 1.0) < 0.5)
    {
      d = floor(d);
    }
    else
    {
      d = ceil(d);
    }

    i = d;
  }
  else if (v.getType() != XmlRpcValue::TypeInt)
  {
    return false;
  }
  else
  {
    i = v;
  }

  return true;
}

bool Node::getParam(const string &key, bool &b, bool use_cache)
{
  if (shuttingDown())
  {
    return false;
  }

  XmlRpcValue v;
  if (!getParam(key, v, use_cache))
    return false;
  if (v.getType() != XmlRpcValue::TypeBoolean)
    return false;
  b = v;
  return true;
}

void Node::paramUpdate(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
  std::string clean_key = cleanName(key);
  ROS_DEBUG("Received parameter update for key [%s]", clean_key.c_str());

  boost::mutex::scoped_lock lock(params_mutex_);

  params_[clean_key] = v;
}

bool Node::splitURI(const string &uri, string &host, int &port)
{
  // skip over the protocol if it's there
  if (uri.substr(0, 7) == string("http://"))
    host = uri.substr(7);
  else if (uri.substr(0, 9) == string("rosrpc://"))
    host = uri.substr(9);
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
//  ROS_INFO("uri = [%s] host = [%s] port = [%s] (%d)\n",
//    uri.c_str(), host.c_str(), port_str.c_str(), port);
  return true;
}

PublicationPtr Node::getPublication(const string &topic)
{
  boost::mutex::scoped_lock lock(advertised_topics_mutex_);

  PublicationPtr t;
  for (V_Publication::iterator i = advertised_topics_.begin();
       !t && i != advertised_topics_.end(); ++i)
  {
    if (((*i)->getName() == topic) && (!(*i)->isDropped()))
    {
      t = *i;
      break;
    }
  }

  return t;
}

void Node::setShutdownMessage(const string &msg)
{
  ROS_WARN("Shutdown request received from the master node.");
  ROS_WARN("Reason given for shutdown: [%s]", msg.c_str());
}

void Node::shutdown()
{
  {
    // Grab all the locks we can so that hopefully nothing happens during/after shutdown
    /// @todo shutdown handling needs further work
    boost::mutex::scoped_lock ss_lock(service_servers_mutex_);
    boost::mutex::scoped_lock adv_lock(advertised_topics_mutex_);
    boost::mutex::scoped_lock subs_lock(subs_mutex_);
    boost::mutex::scoped_lock sc_lock(service_clients_mutex_);
    boost::mutex::scoped_lock shut_lock(shutting_down_mutex_);
    // unregister all of our advertised topics
    if (shutting_down_)
      return; // shutdown already in progress or already happened
    shutting_down_ = true;
  }

  // Stop the TCP server thread
  if(use_server_thread_ && tcpros_server_thread_.get_id() != boost::this_thread::get_id())
  {
    tcpros_server_thread_.join();
    ROS_DEBUG( "tcpros server shut down gracefully");
  }

  tcprosServerFini();

  ROS_DEBUG( "unregistering our advertised services");
  {
    boost::mutex::scoped_lock ss_lock(service_servers_mutex_);

    for (L_ServiceServer::iterator i = service_servers_.begin();
         i != service_servers_.end(); ++i)
    {
      unregisterService((*i)->getName());
      ROS_DEBUG( "shutting down service %s", (*i)->getName().c_str());
      (*i)->drop();
    }
    service_servers_.clear();
  }

  ROS_DEBUG( "unregistering our publishers");
  {
    boost::mutex::scoped_lock adv_lock(advertised_topics_mutex_);

    for (V_Publication::iterator i = advertised_topics_.begin();
         i != advertised_topics_.end(); ++i)
    {
      if(!(*i)->isDropped())
      {
        unregisterPublisher((*i)->getName());
      }
      (*i)->drop();
    }
    advertised_topics_.clear();
  }

  // unregister all of our subscriptions
  ROS_DEBUG( "unregistering our subscribers");
  {
    boost::mutex::scoped_lock subs_lock(subs_mutex_);

    for (L_Subscription::iterator s = subscriptions_.begin(); s != subscriptions_.end(); ++s)
    {
      // Remove us as a subscriber from the master
      unregisterSubscriber((*s)->getName());
      // now, drop our side of the connection
      (*s)->shutdown();
    }
    subscriptions_.clear();
  }

  L_ServiceClient local_service_clients;
  {
    boost::mutex::scoped_lock lock(service_clients_mutex_);
    local_service_clients.swap(service_clients_);
  }

  {
    L_ServiceClient::iterator it = local_service_clients.begin();
    L_ServiceClient::iterator end = local_service_clients.end();
    for (; it != end; ++it)
    {
      (*it)->getConnection()->drop();
    }

    local_service_clients.clear();
  }

  for (vector<MutexedXmlRpcClient>::iterator i = xmlrpc_clients_.begin();
       i != xmlrpc_clients_.end(); ++i)
  {
    for (int wait_count = 0; i->in_use && wait_count < 10; wait_count++)
    {
      ROS_DEBUG("waiting for connection to finish...\n");
      usleep(10000);
    }
    // time's up. UR DONE.
    i->c->close();
    delete i->c;
  }
  xmlrpc_clients_.clear();

  ok_ = false;
  // Free up anybody that's sleeping against simulation time
  Time::shutdown();
}

bool Node::unsubscribe(const std::string &_topic)
{
  std::string topic = mapName(_topic);

  // If we're shutting down, then disconnect_from_botnet() will (or has
  // already) take care of cleaning up subscriptions.
  if(shutting_down_)
  {
    return true;
  }

  SubscriptionPtr sub;

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    if (!unregisterSubscriber(topic))
    {
      ROS_ERROR("couldn't unregister subscriber for topic [%s]", _topic.c_str());
    }

    for (L_Subscription::iterator it = subscriptions_.begin();
         it != subscriptions_.end() && !sub; ++it)
    {
      if ((*it)->getName() == topic)
      {
        sub = *it;
        subscriptions_.erase(it);
        break;
      }
    }
  }

  if(!sub)
  {
    ROS_ERROR( "couldn't find the subscription object in unsubscribe(%s)",
               topic.c_str());
    return false;
  }
  else
  {
    sub->shutdown();
    return true;
  }
}

bool Node::unsubscribe(const Message& _msg)
{
  SubscriptionPtr sub;

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    // dig around to see who is responsible for updating this message
    for (L_Subscription::iterator it = subscriptions_.begin();
         it != subscriptions_.end() && !sub; ++it)
    {
      if ((*it)->updatesMessage(&_msg))
      {
        sub = (*it);

        subscriptions_.erase(it);
        break;
      }
    }

    if (!sub)
    {
      ROS_ERROR("Couldn't find subscriber in unsubscribe(Message)");
      return false;
    }

    if (!unregisterSubscriber(sub->getName()))
    {
      ROS_ERROR("Couldn't unregister subscriber for topic [%s]", sub->getName().c_str());
    }
  }

  sub->shutdown();
  return true;
}

bool Node::unsubscribe(const std::string &_topic, AbstractFunctor *afp)
{
  std::string topic = mapName(_topic);
  SubscriptionPtr sub;
  L_Subscription::iterator it;

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    for (it = subscriptions_.begin();
         it != subscriptions_.end() && !sub; ++it)
    {
      if ((*it)->getName() == topic)
      {
        sub = *it;
        break;
      }
    }
  }

  if (!sub)
  {
    return false;
  }

  sub->removeFunctorMessagePair(afp);

  if (sub->getNumCallbacks() == 0)
  {
    // nobody is left. blow away the subscription.
    {
      boost::mutex::scoped_lock lock(subs_mutex_);

      subscriptions_.erase(it);

      if (!unregisterSubscriber(topic))
      {
        ROS_ERROR("Couldn't unregister subscriber for topic [%s]", _topic.c_str());
      }
    }

    sub->shutdown();
    return true;
  }

  return true;
}

void Node::tcprosAcceptConnection(const TransportTCPPtr& transport)
{
  std::string client_uri = transport->getClientURI();
  ROS_DEBUG("TCPROS received a connection from [%s]", client_uri.c_str());

  ConnectionPtr conn(new Connection());
  addConnection(conn);

  conn->initialize(transport, true, boost::bind(&Node::onConnectionHeaderReceived, this, _1, _2));
}

void Node::addConnection(const ConnectionPtr& conn)
{
  boost::mutex::scoped_lock lock(connections_mutex_);

  connections_.insert(conn);

  conn->addDropListener(boost::bind(&Node::onConnectionDropped, this, _1));
}

size_t Node::numSubscribers(const std::string &_topic)
{
  size_t count = 0;
  std::string topic = mapName(_topic);

  boost::mutex::scoped_lock lock(advertised_topics_mutex_);

  for (V_Publication::iterator t = advertised_topics_.begin();
       t != advertised_topics_.end(); ++t)
  {
    if ((*t)->getName() == topic)
    {
      count = (*t)->getNumSubscribers();
      break;
    }
  }

  return count;
}

PublicationPtr Node::lookupTopic(const std::string& topic)
{
  PublicationPtr ret;
  boost::mutex::scoped_lock lock(advertised_topics_mutex_);

  for (V_Publication::iterator t = advertised_topics_.begin();
       t != advertised_topics_.end(); ++t)
  {
    if ((*t)->getName() == topic)
    {
      ret = *t;
      break;
    }
  }

  return ret;
}

ServiceServerPtr Node::lookupServiceServer(const std::string& service)
{
  boost::mutex::scoped_lock lock(service_servers_mutex_);

  ServiceServerPtr ret;
  for (L_ServiceServer::iterator t = service_servers_.begin();
       t != service_servers_.end(); ++t)
  {
    if ((*t)->getName() == service)
    {
      ret = *t;
      break;
    }
  }

  return ret;
}

ServiceClientPtr Node::createServiceClient(const std::string& service, bool persistent,
                                             const std::string& request_md5sum, const std::string& response_md5sum,
                                             const M_string& header_values)
{
  std::string mapped_name = mapName(service);

  int serv_port;
  std::string serv_host;
  if (!lookupService(mapped_name, serv_host, serv_port))
  {
    return ServiceClientPtr();
  }

  TransportTCPPtr transport(new TransportTCP(&poll_set_));
  if (transport->connect(serv_host, serv_port))
  {
    ConnectionPtr connection(new Connection());
    addConnection(connection);

    ServiceClientPtr client(new ServiceClient(mapped_name, persistent, request_md5sum, response_md5sum, header_values));

    {
      boost::mutex::scoped_lock lock(service_clients_mutex_);
      service_clients_.push_back(client);
    }

    connection->initialize(transport, false, HeaderReceivedFunc());
    client->initialize(connection);

    return client;
  }

  ROS_ERROR("Failed to connect to service [%s] (mapped=[%s]) at [%s:%d]", service.c_str(), mapped_name.c_str(), serv_host.c_str(), serv_port);

  return ServiceClientPtr();
}

void Node::removeServiceClient(const ServiceClientPtr& client)
{
  boost::mutex::scoped_lock lock(service_clients_mutex_);

  L_ServiceClient::iterator it = std::find(service_clients_.begin(), service_clients_.end(), client);
  if (it != service_clients_.end())
  {
    service_clients_.erase(it);
  }
}

bool Node::lookupService(const string &name_, string &serv_host,
                             int &serv_port)
{
  XmlRpcValue args, result, payload;
  args[0] = getName();
  args[1] = name_;
  if (!executeMasterXmlrpc("lookupService", args, result, payload, false))
    return false;

  string serv_uri(payload);
  if (!serv_uri.length()) // shouldn't happen. but let's be sure.
  {
    ROS_ERROR("lookupService: Empty server URI returned from master");

    return false;
  }

  if (!splitURI(serv_uri, serv_host, serv_port))
  {
    ROS_ERROR("lookupService: Bad service uri [%s]", serv_uri.c_str());

    return false;
  }

  return true;
}

XmlRpcClient *Node::getXMLRPCClient(const std::string &host, const int port,
                                      const std::string &uri)
{
  // go through our vector of clients and grab the first available one
  XmlRpcClient *c = NULL;

  boost::mutex::scoped_lock lock(xmlrpc_clients_mutex_);

  for (vector<MutexedXmlRpcClient>::iterator i = xmlrpc_clients_.begin();
       !c && i != xmlrpc_clients_.end(); )
  {
    if (!i->in_use)
    {
      // see where it's pointing
      if (i->c->getHost() == host &&
          i->c->getPort() == port &&
          i->c->getUri()  == uri)
      {
        // hooray, it's pointing at our destination. re-use it.
        c = i->c;
        i->in_use = true;
        i->last_use_time = Time::now().toSec();
        break;
      }
      else if (i->last_use_time + MutexedXmlRpcClient::zombie_time
               < Time::now().toSec())
      {
        // toast this guy. he's dead and nobody is reusing him.
        delete i->c;
        i = xmlrpc_clients_.erase(i);
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
    MutexedXmlRpcClient mc(c);
    mc.in_use = true;
    mc.last_use_time = Time::now().toSec();
    xmlrpc_clients_.push_back(mc);
    //ROS_INFO("%d xmlrpc clients allocated\n", xmlrpc_clients.size());
  }
  // ONUS IS ON THE RECEIVER TO UNSET THE IN_USE FLAG
  // by calling releaseXMLRPCClient
  return c;
}

void Node::releaseXMLRPCClient(XmlRpcClient *c)
{
  boost::mutex::scoped_lock lock(xmlrpc_clients_mutex_);

  for (vector<MutexedXmlRpcClient>::iterator i = xmlrpc_clients_.begin();
       i != xmlrpc_clients_.end(); ++i)
  {
    if (c == i->c)
    {
      i->in_use = false;
      break;
    }
  }
}

void Node::getBusStats(XmlRpcValue &stats)
{
  XmlRpcValue publish_stats, subscribe_stats, service_stats;
  // force these guys to be arrays, even if we don't populate them
  publish_stats.setSize(0);
  subscribe_stats.setSize(0);
  service_stats.setSize(0);

  uint32_t pidx = 0;
  {
    boost::mutex::scoped_lock lock(advertised_topics_mutex_);
    for (V_Publication::iterator t = advertised_topics_.begin();
         t != advertised_topics_.end(); ++t)
    {
      publish_stats[pidx++] = (*t)->getStats();
    }
  }

  {
    uint32_t sidx = 0;

    boost::mutex::scoped_lock lock(subs_mutex_);
    for (L_Subscription::iterator t = subscriptions_.begin(); t != subscriptions_.end(); ++t)
    {
      subscribe_stats[sidx++] = (*t)->getStats();
    }
  }

  stats[0] = publish_stats;
  stats[1] = subscribe_stats;
  stats[2] = service_stats;
}

void Node::getBusInfo(XmlRpcValue &info)
{
  // force these guys to be arrays, even if we don't populate them
  info.setSize(0);

  {
    boost::mutex::scoped_lock lock(advertised_topics_mutex_);

    for (V_Publication::iterator t = advertised_topics_.begin();
         t != advertised_topics_.end(); ++t)
    {
      (*t)->getInfo(info);
    }
  }

  {
    boost::mutex::scoped_lock lock(subs_mutex_);

    for (L_Subscription::iterator t = subscriptions_.begin(); t != subscriptions_.end(); ++t)
    {
      (*t)->getInfo(info);
    }
  }
}

bool Node::checkMaster()
{
  XmlRpcValue args, result, payload;
  args[0] = name_;
  return executeMasterXmlrpc("getTime", args, result, payload, false);
}

bool Node::_shutdownMaster()
{
  XmlRpcValue args, result, payload;
  args[0] = name_;
  args[1] = string("testing");
  return executeMasterXmlrpc("shutdown", args, result, payload, false);
}

void Node::timeCallback()
{
  Time::setNow(time_msg_.rostime);
}
