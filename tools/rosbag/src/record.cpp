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

void printUsage() {
    fprintf(stderr, "Usage: record [options] TOPIC1 [TOPIC2 TOPIC3...]\n"
                    "  record logs ROS message data to a file.\n");
    fprintf(stderr, "Options:\n");
    fprintf(stderr, " -c <num>    : Only receive <num> messages on each topic\n");
    fprintf(stderr, " -f <prefix> : Prepend file prefix to beginning of bag name (name will always end with date stamp)\n");
    fprintf(stderr, " -F <fname>  : Record to a file named exactly <fname>.bag\n");
    fprintf(stderr, " -a          : Record all published messages.\n");
    fprintf(stderr, " -v          : Display a message every time a message is received on a topic\n");
    fprintf(stderr, " -m          : Maximize internal buffer size in MB (Default: 256MB)  0 = infinite.\n");
    fprintf(stderr, " -s          : (EXPERIMENTAL) Enable snapshot recording (don't write to file unless triggered)\n");
    fprintf(stderr, " -t          : (EXPERIMENTAL) Trigger snapshot recording\n");
    fprintf(stderr, " -h          : Display this help message\n");
}

//! Parse the command-line arguments for recorder options
rosbag::RecorderOptions parseOptions(int argc, char** argv) {
    rosbag::RecorderOptions opts;

    int option_char;
    while ((option_char = getopt(argc, argv, "f:F:c:m:S:astvzj")) != -1) {
        switch (option_char) {
        case 'f': opts.prefix      = std::string(optarg); break;
        case 'F': opts.prefix      = std::string(optarg); opts.append_date = false; break;
        case 'c': opts.limit       = atoi(optarg); break;
        case 'a': opts.record_all  = true; break;
        case 's': opts.snapshot    = true; break;
        case 'v': opts.verbose     = true; break;
        case 't': opts.trigger     = true; break;
        //case 'z': opts.compression = rosbag::compression::ZLIB; break;
        case 'j': opts.compression = rosbag::compression::BZ2; break;
        case 'm': {
            int m = atoi(optarg);
            if (m < 0)
                throw ros::Exception("Buffer size must be 0 or positive");
            opts.buffer_size = 1048576 * m;
            break;
        }
        case 'S': {
            int S = atoi(optarg);
            if (S < 0)
                throw ros::Exception("Split size must be 0 or positive");
            opts.split_size = 1048576 * S;
            break;
        }
        }
    }

    // Every non-option argument is assumed to be a topic
    for (; optind < argc; optind++)
        opts.topics.push_back(std::string(argv[optind]));

    return opts;
}

int main(int argc, char** argv) {
    // Parse the command-line options
    rosbag::RecorderOptions opts;
    try {
        opts = parseOptions(argc, argv);
    }
    catch (ros::Exception const& ex) {
        fprintf(stderr, "Error reading options: %s", ex.what());
        return 1;
    }

    ros::init(argc, argv, "record", ros::init_options::AnonymousName);

    // Run the recorder
	rosbag::Recorder recorder(opts);
	int result = recorder.run();

	return result;
}
