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

#include "ros/param.h"
#include "ros/master.h"
#include "ros/xmlrpc_manager.h"
#include "ros/this_node.h"
#include "ros/names.h"

#include <ros/console.h>

#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>

namespace ros
{

namespace param
{

typedef std::map<std::string, XmlRpc::XmlRpcValue> M_Param;
M_Param g_params;
boost::mutex g_params_mutex;
S_string g_subscribed_params;

void set(const std::string &key, const XmlRpc::XmlRpcValue &v)
{
  std::string mapped_key = ros::names::resolve(key);

  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = mapped_key;
  params[2] = v;

  {
    // Lock around the execute to the master in case we get a parameter update on this value between
    // executing on the master and setting the parameter in the g_params list.
    boost::mutex::scoped_lock lock(g_params_mutex);

    if (master::execute("setParam", params, result, payload, true))
    {
      // Update our cached params list now so that if get() is called immediately after param::set()
      // we already have the cached state and our value will be correct
      if (g_subscribed_params.find(mapped_key) != g_subscribed_params.end())
      {
        g_params[mapped_key] = v;
      }
    }
  }
}

void set(const std::string &key, const std::string &s)
{
  // construct xmlrpc_c::value object of the std::string and
  // call param::set(key, xmlvalue);
  XmlRpc::XmlRpcValue v(s);
  param::set(key, v);
}

void set(const std::string &key, const char* s)
{
  // construct xmlrpc_c::value object of the std::string and
  // call param::set(key, xmlvalue);
  std::string sxx = std::string(s);
  XmlRpc::XmlRpcValue v(sxx);
  ROS_DEBUG("param::set(): set %s, type %d\n", key.c_str(), v.getType());
  param::set(key, v);
}

void set(const std::string &key, double d)
{
  XmlRpc::XmlRpcValue v(d);
  param::set(key, v);
}

void set(const std::string &key, int i)
{
  XmlRpc::XmlRpcValue v(i);
  param::set(key, v);
}

void set(const std::string& key, bool b)
{
  XmlRpc::XmlRpcValue v(b);
  param::set(key, v);
}

bool has(const std::string &key)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = ros::names::resolve(key);
  //params[1] = key;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!master::execute("hasParam", params, result, payload, false))
  {
    return false;
  }

  return payload;
}

bool del(const std::string &key)
{
  std::string mapped_key = ros::names::resolve(key);

  {
    boost::mutex::scoped_lock lock(g_params_mutex);

    S_string::iterator sub_it = g_subscribed_params.find(mapped_key);
    if (sub_it != g_subscribed_params.end())
    {
      g_subscribed_params.erase(sub_it);

      M_Param::iterator param_it = g_params.find(mapped_key);
      if (param_it != g_params.end())
      {
        g_params.erase(param_it);
      }
    }
  }

  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = mapped_key;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!master::execute("deleteParam", params, result, payload, false))
  {
    return false;
  }

  return true;
}

bool get(const std::string &key, XmlRpc::XmlRpcValue &v, bool use_cache)
{
  std::string mapped_key = ros::names::resolve(key);

  if (use_cache)
  {
    boost::mutex::scoped_lock lock(g_params_mutex);

    if (g_subscribed_params.find(mapped_key) != g_subscribed_params.end())
    {
      M_Param::iterator it = g_params.find(mapped_key);
      if (it != g_params.end())
      {
        ROS_DEBUG("Using cached parameter value for key [%s]", mapped_key.c_str());

        v = it->second;
        return true;
      }
    }
    else
    {
      // parameter we've never seen before, register for update from the master
      if (g_subscribed_params.insert(mapped_key).second)
      {
        XmlRpc::XmlRpcValue params, result, payload;
        params[0] = this_node::getName();
        params[1] = XMLRPCManager::instance()->getServerURI();
        params[2] = mapped_key;

        if (!master::execute("subscribeParam", params, result, payload, false))
        {
          g_subscribed_params.erase(mapped_key);
        }
        else
        {
          ROS_DEBUG("Subscribed to parameter [%s]", mapped_key.c_str());
        }
      }
    }
  }

  XmlRpc::XmlRpcValue params, result;
  params[0] = this_node::getName();
  params[1] = mapped_key;

  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!master::execute("getParam", params, result, v, false))
  {
    return false;
  }

  if (use_cache && v.valid())
  {
    boost::mutex::scoped_lock lock(g_params_mutex);

    ROS_DEBUG("Caching parameter [%s] with value type [%d]", mapped_key.c_str(), v.getType());
    g_params[mapped_key] = v;
  }

  return true;
}

bool get(const std::string &key, std::string &s, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!get(key, v, use_cache))
    return false;
  if (v.getType() != XmlRpc::XmlRpcValue::TypeString)
    return false;
  s = std::string(v);
  return true;
}

bool get(const std::string &key, double &d, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!get(key, v, use_cache))
  {
    return false;
  }

  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    d = (int)v;
  }
  else if (v.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    return false;
  }
  else
  {
    d = v;
  }

  return true;
}

bool get(const std::string &key, int &i, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!get(key, v, use_cache))
  {
    return false;
  }

  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
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
  else if (v.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    return false;
  }
  else
  {
    i = v;
  }

  return true;
}

bool get(const std::string &key, bool &b, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!get(key, v, use_cache))
    return false;
  if (v.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    return false;
  b = v;
  return true;
}

void update(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
  std::string clean_key = names::clean(key);
  ROS_DEBUG("Received parameter update for key [%s]", clean_key.c_str());

  boost::mutex::scoped_lock lock(g_params_mutex);

  g_params[clean_key] = v;
}

void paramUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  result[0] = 1;
  result[1] = std::string("");
  result[2] = 0;

  param::update((std::string)params[1], params[2]);
}

void init(const M_string& remappings)
{
  M_string::const_iterator it = remappings.begin();
  M_string::const_iterator end = remappings.end();
  for (; it != end; ++it)
  {
    const std::string& name = it->first;
    const std::string& param = it->second;

    if (name.size() < 2)
    {
      continue;
    }

    if (name[0] == '_' && name[1] != '_')
    {
      std::string local_name = "~" + name.substr(1);

      bool success = false;

      try
      {
        int32_t i = boost::lexical_cast<int32_t>(param);
        param::set(names::resolve(local_name), i);
        success = true;
      }
      catch (boost::bad_lexical_cast&)
      {

      }

      if (success)
      {
        continue;
      }

      try
      {
        double d = boost::lexical_cast<double>(param);
        param::set(names::resolve(local_name), d);
        success = true;
      }
      catch (boost::bad_lexical_cast&)
      {

      }

      if (success)
      {
        continue;
      }

      if (param == "true" || param == "True" || param == "TRUE")
      {
        param::set(names::resolve(local_name), true);
      }
      else if (param == "false" || param == "False" || param == "FALSE")
      {
        param::set(names::resolve(local_name), false);
      }
      else
      {
        param::set(names::resolve(local_name), param);
      }
    }
  }

  XMLRPCManager::instance()->bind("paramUpdate", paramUpdateCallback);
}

} // namespace param

} // namespace ros
