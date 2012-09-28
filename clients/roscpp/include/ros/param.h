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

#ifndef ROSCPP_PARAM_H
#define ROSCPP_PARAM_H

#include "forwards.h"
#include "common.h"
#include "XmlRpcValue.h"

namespace ros
{

/**
 * \brief Contains functions which allow you to query the parameter server
 */
namespace param
{

/** \brief Set an arbitrary XML/RPC value on the parameter server.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param v The value to be inserted.
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL void set(const std::string& key, const XmlRpc::XmlRpcValue& v);
/** \brief Set a string value on the parameter server.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param s The value to be inserted.
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL void set(const std::string& key, const std::string& s);
/** \brief Set a string value on the parameter server.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param s The value to be inserted.
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL void set(const std::string& key, const char* s);
/** \brief Set a double value on the parameter server.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param d The value to be inserted.
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL void set(const std::string& key, double d);
/** \brief Set a integer value on the parameter server.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param i The value to be inserted.
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL void set(const std::string& key, int i);
/** \brief Set a integer value on the parameter server.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param b The value to be inserted.
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL void set(const std::string& key, bool b);

/** \brief Get a string value from the parameter server.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param[out] s Storage for the retrieved value.
 *
 * \return true if the parameter value was retrieved, false otherwise
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool get(const std::string& key, std::string& s);
/** \brief Get a double value from the parameter server.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param[out] d Storage for the retrieved value.
 *
 * \return true if the parameter value was retrieved, false otherwise
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool get(const std::string& key, double& d);
/** \brief Get a integer value from the parameter server.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param[out] i Storage for the retrieved value.
 *
 * \return true if the parameter value was retrieved, false otherwise
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool get(const std::string& key, int& i);
/** \brief Get a boolean value from the parameter server.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param[out] b Storage for the retrieved value.
 *
 * \return true if the parameter value was retrieved, false otherwise
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool get(const std::string& key, bool& b);
/** \brief Get an arbitrary XML/RPC value from the parameter server.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param[out] v Storage for the retrieved value.
 *
 * \return true if the parameter value was retrieved, false otherwise
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool get(const std::string& key, XmlRpc::XmlRpcValue& v);

/** \brief Get a string value from the parameter server, with local caching
 *
 * This function will cache parameters locally, and subscribe for updates from
 * the parameter server.  Once the parameter is retrieved for the first time
 * no subsequent getCached() calls with the same key will query the master --
 * they will instead look up in the local cache.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param[out] s Storage for the retrieved value.
 *
 * \return true if the parameter value was retrieved, false otherwise
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool getCached(const std::string& key, std::string& s);
/** \brief Get a double value from the parameter server, with local caching
 *
 * This function will cache parameters locally, and subscribe for updates from
 * the parameter server.  Once the parameter is retrieved for the first time
 * no subsequent getCached() calls with the same key will query the master --
 * they will instead look up in the local cache.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param[out] d Storage for the retrieved value.
 *
 * \return true if the parameter value was retrieved, false otherwise
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool getCached(const std::string& key, double& d);
/** \brief Get a integer value from the parameter server, with local caching
 *
 * This function will cache parameters locally, and subscribe for updates from
 * the parameter server.  Once the parameter is retrieved for the first time
 * no subsequent getCached() calls with the same key will query the master --
 * they will instead look up in the local cache.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param[out] i Storage for the retrieved value.
 *
 * \return true if the parameter value was retrieved, false otherwise
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool getCached(const std::string& key, int& i);
/** \brief Get a boolean value from the parameter server, with local caching
 *
 * This function will cache parameters locally, and subscribe for updates from
 * the parameter server.  Once the parameter is retrieved for the first time
 * no subsequent getCached() calls with the same key will query the master --
 * they will instead look up in the local cache.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param[out] b Storage for the retrieved value.
 *
 * \return true if the parameter value was retrieved, false otherwise
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool getCached(const std::string& key, bool& b);
/** \brief Get an arbitrary XML/RPC value from the parameter server, with local caching
 *
 * This function will cache parameters locally, and subscribe for updates from
 * the parameter server.  Once the parameter is retrieved for the first time
 * no subsequent getCached() calls with the same key will query the master --
 * they will instead look up in the local cache.
 *
 * \param key The key to be used in the parameter server's dictionary
 * \param[out] v Storage for the retrieved value.
 *
 * \return true if the parameter value was retrieved, false otherwise
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool getCached(const std::string& key, XmlRpc::XmlRpcValue& v);

/** \brief Check whether a parameter exists on the parameter server.
 *
 * \param key The key to check.
 *
 * \return true if the parameter exists, false otherwise
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool has(const std::string& key);
/** \brief Delete a parameter from the parameter server.
 *
 * \param key The key to delete.
 *
 * \return true if the deletion succeeded, false otherwise.
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool del(const std::string& key);

/** \brief Search up the tree for a parameter with a given key
 *
 * This function parameter server's searchParam feature to search up the tree for
 * a parameter.  For example, if the parameter server has a parameter [/a/b]
 * and you specify the namespace [/a/c/d], searching for the parameter "b" will
 * yield [/a/b].  If [/a/c/d/b] existed, that parameter would be returned instead.
 *
 * \param ns The namespace to begin the search in
 * \param key the parameter to search for
 * \param [out] result the found value (if any)
 *
 * \return true if the parameter was found, false otherwise.
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool search(const std::string& ns, const std::string& key, std::string& result);

/** \brief Search up the tree for a parameter with a given key.  This version defaults to starting in
 * the current node's name
 *
 * This function parameter server's searchParam feature to search up the tree for
 * a parameter.  For example, if the parameter server has a parameter [/a/b]
 * and you specify the namespace [/a/c/d], searching for the parameter "b" will
 * yield [/a/b].  If [/a/c/d/b] existed, that parameter would be returned instead.
 *
 * \param key the parameter to search for
 * \param [out] result the found value (if any)
 *
 * \return true if the parameter was found, false otherwise.
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
ROSCPP_DECL bool search(const std::string& key, std::string& result);

/** \brief Assign value from parameter server, with default.
 *
 * This method tries to retrieve the indicated parameter value from the
 * parameter server, storing the result in param_val.  If the value
 * cannot be retrieved from the server, default_val is used instead.
 *
 * \param param_name The key to be searched on the parameter server.
 * \param[out] param_val Storage for the retrieved value.
 * \param default_val Value to use if the server doesn't contain this
 * parameter.
 * \throws InvalidNameException if the key is not a valid graph resource name
 */
template<typename T>
void param(const std::string& param_name, T& param_val, const T& default_val)
{
  if (has(param_name))
  {
    if (get(param_name, param_val))
    {
      return;
    }
  }

  param_val = default_val;
}

} // namespace param

} // namespace param

#endif // ROSCPP_PARAM_H
