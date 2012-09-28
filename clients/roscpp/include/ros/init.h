/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef ROSCPP_INIT_H
#define ROSCPP_INIT_H

#include "ros/forwards.h"
#include "ros/spinner.h"
#include "common.h"

namespace ros
{

namespace init_options
{
/**
 * \brief Flags for ROS initialization
 */
enum InitOption
{
  /**
   * Don't install a SIGINT handler.  You should install your own SIGINT handler in this
   * case, to ensure that the node gets shutdown correctly when it exits.
   */
  NoSigintHandler = 1 << 0,
  /** \brief Anonymize the node name.  Adds a random number to the end of your node's name, to make it unique.
   */
  AnonymousName = 1 << 1,
  /**
   * \brief Don't broadcast rosconsole output to the /rosout topic
   */
  NoRosout = 1 << 2,
};
}
typedef init_options::InitOption InitOption;

/** @brief ROS initialization function.
 *
 * This function will parse any ROS arguments (e.g., topic name
 * remappings), and will consume them (i.e., argc and argv may be modified
 * as a result of this call).
 *
 * Use this version if you are using the NodeHandle API
 *
 * \param argc
 * \param argv
 * \param name Name of this node.  The name must be a base name, ie. it cannot contain namespaces.
 * \param options [optional] Options to start the node with (a set of bit flags from \ref ros::init_options)
 * \throws InvalidNodeNameException If the name passed in is not a valid "base" name
 *
 */
ROSCPP_DECL void init(int &argc, char **argv, const std::string& name, uint32_t options = 0);

/**
 * \brief alternate ROS initialization function.
 *
 * \param remappings A map<string, string> where each one constitutes a name remapping, or one of the special remappings like __name, __master, __ns, etc.
 * \param name Name of this node.  The name must be a base name, ie. it cannot contain namespaces.
 * \param options [optional] Options to start the node with (a set of bit flags from \ref ros::init_options)
 * \throws InvalidNodeNameException If the name passed in is not a valid "base" name
 */
ROSCPP_DECL void init(const M_string& remappings, const std::string& name, uint32_t options = 0);

/**
 * \brief alternate ROS initialization function.
 *
 * \param remappings A vector<pair<string, string> > where each one constitutes a name remapping, or one of the special remappings like __name, __master, __ns, etc.
 * \param name Name of this node.  The name must be a base name, ie. it cannot contain namespaces.
 * \param options [optional] Options to start the node with (a set of bit flags from \ref ros::init_options)
 * \throws InvalidNodeNameException If the name passed in is not a valid "base" name
 */
ROSCPP_DECL void init(const VP_string& remapping_args, const std::string& name, uint32_t options = 0);

/**
 * \brief Returns whether or not ros::init() has been called
 */
ROSCPP_DECL bool isInitialized();
/**
 * \brief Returns whether or not ros::shutdown() has been (or is being) called
 */
ROSCPP_DECL bool isShuttingDown();

/** \brief Enter simple event loop
 *
 * This method enters a loop, processing callbacks.  This method should only be used
 * if the NodeHandle API is being used.
 *
 * This method is mostly useful when your node does all of its work in
 * subscription callbacks.  It will not process any callbacks that have been assigned to
 * custom queues.
 *
 */
ROSCPP_DECL void spin();

/** \brief Enter simple event loop
 *
 * This method enters a loop, processing callbacks.  This method should only be used
 * if the NodeHandle API is being used.
 *
 * This method is mostly useful when your node does all of its work in
 * subscription callbacks.  It will not process any callbacks that have been assigned to
 * custom queues.
 *
 * \param spinner a spinner to use to call callbacks.  Two default implementations are available,
 * SingleThreadedSpinner and MultiThreadedSpinner
 */
ROSCPP_DECL void spin(Spinner& spinner);
/**
 * \brief Process a single round of callbacks.
 *
 * This method is useful if you have your own loop running and would like to process
 * any callbacks that are available.  This is equivalent to calling callAvailable() on the
 * global CallbackQueue.  It will not process any callbacks that have been assigned to
 * custom queues.
 */
ROSCPP_DECL void spinOnce();

/**
 * \brief Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown(), or similar.
 */
ROSCPP_DECL void waitForShutdown();

/** \brief Check whether it's time to exit.
 *
 * ok() becomes false once ros::shutdown() has been called and is finished
 *
 * \return true if we're still OK, false if it's time to exit
 */
ROSCPP_DECL bool ok();
/**
 * \brief Disconnects everything and unregisters from the master.  It is generally not
 * necessary to call this function, as the node will automatically shutdown when all
 * NodeHandles destruct.  However, if you want to break out of a spin() loop explicitly,
 * this function allows that.
 */
ROSCPP_DECL void shutdown();

/**
 * \brief Request that the node shut itself down from within a ROS thread
 *
 * This method signals a ROS thread to call shutdown().
 */
ROSCPP_DECL void requestShutdown();

/**
 * \brief Actually starts the internals of the node (spins up threads, starts the network polling and xmlrpc loops,
 * connects to internal subscriptions like /clock, starts internal service servers, etc.).
 *
 * Usually unnecessary to call manually, as it is automatically called by the creation of the first NodeHandle if
 * the node has not already been started.  If you would like to prevent the automatic shutdown caused by the last
 * NodeHandle going out of scope, call this before any NodeHandle has been created (e.g. immediately after init())
 */
ROSCPP_DECL void start();
/**
 * \brief Returns whether or not the node has been started through ros::start()
 */
ROSCPP_DECL bool isStarted();

/**
 * \brief Returns a pointer to the global default callback queue.
 *
 * This is the queue that all callbacks get added to unless a different one is specified, either in the NodeHandle
 * or in the individual NodeHandle::subscribe()/NodeHandle::advertise()/etc. functions.
 */
ROSCPP_DECL CallbackQueue* getGlobalCallbackQueue();

/**
 * \brief returns a vector of program arguments that do not include any ROS remapping arguments.  Useful if you need
 * to parse your arguments to determine your node name
 *
 * \param argc the number of command-line arguments
 * \param argv the command-line arguments
 * \param [out] args_out Output args, stripped of any ROS args
 */
ROSCPP_DECL void removeROSArgs(int argc, const char* const* argv, V_string& args_out);

}

#endif
