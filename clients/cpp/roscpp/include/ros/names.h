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

#ifndef ROSCPP_NAMES_H
#define ROSCPP_NAMES_H

#include "forwards.h"
#include "common.h"

namespace ros
{

/**
 * \brief Contains functions which allow you to manipulate ROS names
 */
namespace names
{

/**
 * \brief Cleans a graph resource name: removes double slashes, trailing slash
 */
ROSCPP_DECL std::string clean(const std::string& name);
/**
 * \brief Resolve a graph resource name into a fully qualified graph resource name
 *
 * See http://www.ros.org/wiki/Names for more details
 *
 * \param name Name to resolve
 * \param remap Whether or not to apply remappings to the name
 * \throws InvalidNameException if the name passed is not a valid graph resource name
 */
ROSCPP_DECL std::string resolve(const std::string& name, bool remap = true);
/**
 * \brief Resolve a graph resource name into a fully qualified graph resource name
 *
 * See http://www.ros.org/wiki/Names for more details
 *
 * \param ns Namespace to use in resolution
 * \param name Name to resolve
 * \param remap Whether or not to apply remappings to the name
 * \throws InvalidNameException if the name passed is not a valid graph resource name
 */
ROSCPP_DECL std::string resolve(const std::string& ns, const std::string& name, bool remap = true);
/**
 * \brief Append one name to another
 */
ROSCPP_DECL std::string append(const std::string& left, const std::string& right);
/**
 * \brief Apply remappings to a name
 * \throws InvalidNameException if the name passed is not a valid graph resource name
 */
ROSCPP_DECL std::string remap(const std::string& name);
/**
 * \brief Validate a name against the name spec
 */
ROSCPP_DECL bool validate(const std::string& name, std::string& error);

ROSCPP_DECL const M_string& getRemappings();
ROSCPP_DECL const M_string& getUnresolvedRemappings();

/**
 * \brief Get the parent namespace of a name
 * \param name The namespace of which to get the parent namespace.  
 * \throws InvalidNameException if the name passed is not a valid graph resource name
 */
ROSCPP_DECL std::string parentNamespace(const std::string& name);

} // namespace names

} // namespace ros

#endif // ROSCPP_NAMES_H
