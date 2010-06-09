/*********************************************************************
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
********************************************************************/

#include "rosbag/player.h"
#include "boost/program_options.hpp"

namespace po = boost::program_options;

rosbag::PlayerOptions parseOptions(int argc, char** argv) {
    rosbag::PlayerOptions opts;

    po::options_description desc("Allowed options");

    desc.add_options()
      ("quiet,q", "suppress console output")
      ("immediate,i", "play back all messages without waiting")
      ("pause", "start in paused mode")
      ("queue", po::value<int>()->default_value(0), "use an outgoing queue of size SIZE")
      ("clock", "publish the clock time")
      ("hz", po::value<float>()->default_value(100.0), "use a frequency of HZ when publishing clock time")
      ("delay,d", po::value<float>()->default_value(0.2), "sleep SEC seconds after every advertise call")
      ("rate,r", po::value<float>()->default_value(1.0), "multiply the publish rate by FACTOR")
      ("start,s", po::value<float>()->default_value(0.0), "start SEC seconds into the bag files")
      ("loop,l", "loop playback")
      ("try-future-version", "still try to open a bag file, even if the version is not known to the player")
      ("input-file", po::value< std::vector<std::string> >(), "input files");
    
    po::positional_options_description p;
    p.add("input-file", -1);
    
    po::variables_map vm;
    
    try 
    {
      po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    } catch (boost::program_options::invalid_command_line_syntax& e)
    {
      throw ros::Exception(e.what());
    }  catch (boost::program_options::unknown_option& e)
    {
      throw ros::Exception(e.what());
    }

    if (vm.count("quiet"))
      opts.quiet = true;
    if (vm.count("immediate"))
      opts.at_once = true;
    if (vm.count("pause"))
      opts.start_paused = true;
    if (vm.count("queue"))
      opts.queue_size = vm["queue"].as<int>();
    if (vm.count("hz"))
      opts.bag_time_frequency = vm["hz"].as<float>();
    if (!vm.count("clock"))
      opts.bag_time_frequency = 0.0;
    if (vm.count("delay"))
      opts.advertise_sleep = ros::WallDuration(vm["delay"].as<float>());
    if (vm.count("rate"))
      opts.time_scale = vm["rate"].as<float>();
    if (vm.count("start"))
    {
      opts.time = vm["start"].as<float>();
      opts.has_time = true;
    }
    if (vm.count("loop"))
      opts.loop = true;


    if (vm.count("input-file"))
    {
      std::vector<std::string> bags = vm["input-file"].as< std::vector<std::string> >();
      for (std::vector<std::string>::iterator i = bags.begin();
           i != bags.end();
           i++)
          opts.bags.push_back(*i);
    }
            
    return opts;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "play", ros::init_options::AnonymousName);

    // Parse the command-line options
    rosbag::PlayerOptions opts;
    try {
        opts = parseOptions(argc, argv);
    }
    catch (ros::Exception const& ex) {
        ROS_ERROR("Error reading options: %s", ex.what());
        return 1;
    }

    rosbag::Player player(opts);

    try {
      player.publish();
    }
    catch (std::runtime_error& e) {
      ROS_FATAL("%s", e.what());
      return 1;
    }
    
    return 0;
}
