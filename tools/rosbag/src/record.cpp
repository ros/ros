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

#include "rosbag/recorder.h"
#include "rosbag/exceptions.h"

#include "boost/program_options.hpp"

namespace po = boost::program_options;

//! Parse the command-line arguments for recorder options
rosbag::RecorderOptions parseOptions(int argc, char** argv) {
    rosbag::RecorderOptions opts;

    po::options_description desc("Allowed options");

    desc.add_options()
      ("all,a", "record all topics")
      ("regex,e", "match topics using regular expressions")
      ("exclude,x", po::value<std::string>(), "exclude topics matching regular expressions")
      ("quiet,q", "suppress console output")
      ("output-prefix,o", po::value<std::string>(), "prepend PREFIX to beginning of bag name")
      ("output-name,O", po::value<std::string>(), "record bagnamed NAME.bag")
      ("split", po::value<int>()->default_value(0), "split bag file into files of size SIZE in MB")
      ("buffsize,b", po::value<int>()->default_value(256), "Use an internal buffer of SIZE MB (Default: 256)")
      ("limit,l", po::value<int>()->default_value(0), "Only record NUM messages on each topic")
      ("bz2,j", "use BZ2 compression")
      ("topic", po::value< std::vector<std::string> >(), "topic to record");
    
    po::positional_options_description p;
    p.add("topic", -1);
    
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

    if (vm.count("all"))
      opts.record_all = true;
    if (vm.count("regex"))
      opts.regex = true;
    if (vm.count("exclude"))
    {
      opts.do_exclude = true;
      opts.exclude_regex = vm["exclude"].as<std::string>();
    }
    if (vm.count("quiet"))
      opts.quiet = true;
    if (vm.count("output-prefix"))
    {
      opts.prefix = vm["output-prefix"].as<std::string>();
      opts.append_date = true;
    }
    if (vm.count("output-name"))
    {
      opts.prefix = vm["output-name"].as<std::string>();
      opts.append_date = false;
    }
    if (vm.count("split"))
    {
      int S = vm["split"].as<int>();
      if (S < 0)
        throw ros::Exception("Split size must be 0 or positive");
      opts.split_size = 1048576 * S;
    }
    if (vm.count("buffsize"))
    {
      int m = vm["buffsize"].as<int>();
      if (m < 0)
        throw ros::Exception("Buffer size must be 0 or positive");
      opts.buffer_size = 1048576 * m;
    }
    if (vm.count("limit"))
    {
      opts.limit = vm["limit"].as<int>();
    }
    if (vm.count("bz2"))
    {
      opts.compression = rosbag::compression::BZ2;
    }

    // Every non-option argument is assumed to be a topic
    if (vm.count("topic"))
    {
      std::vector<std::string> bags = vm["topic"].as< std::vector<std::string> >();
      for (std::vector<std::string>::iterator i = bags.begin();
           i != bags.end();
           i++)
        opts.topics.push_back(std::string(argv[optind]));
    }


    // check that argument combinations make sense
    if(opts.exclude_regex.size() > 0 &&
            !(opts.record_all || opts.regex)) {
        fprintf(stderr, "Warning: Exclusion regex given, but no topics to subscribe to.\n"
                "Adding implicit 'record all'.");
        opts.record_all = true;
    }

    return opts;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "record", ros::init_options::AnonymousName);

    // Parse the command-line options
    rosbag::RecorderOptions opts;
    try {
        opts = parseOptions(argc, argv);
    }
    catch (ros::Exception const& ex) {
        ROS_ERROR("Error reading options: %s", ex.what());
        return 1;
    }
    catch(boost::regex_error const& ex) {
        ROS_ERROR("Error reading options: %s\n", ex.what());
        return 1;
    }

    // Run the recorder
	rosbag::Recorder recorder(opts);
	int result = recorder.run();

	return result;
}
