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

void printUsage() {
    fprintf(stderr, "Usage: play [options] BAG1 [BAG2]\n");
    fprintf(stderr, "Options:\n");
    fprintf(stderr, " -n\tdisable display of current log time\n");
    fprintf(stderr, " -c\tcheck the contents of the bag without playing back\n");
    fprintf(stderr, " -a\tplayback all messages without waiting\n");
    fprintf(stderr, " -b hz\tpublish the bag time at frequence <hz>\n");
    fprintf(stderr, " -p\tstart in paused mode\n");
    fprintf(stderr, " -r\tincrease the publish rate ba a factor <rate_change>\n");
    fprintf(stderr, " -s sec\tsleep <sec> sleep duration after every advertise call (to allow subscribers to connect)\n");
    fprintf(stderr, " -t sec\tstart <sec> seconds into the files\n");
    fprintf(stderr, " -q sz\tUse an outgoing queue of size <sz> (defaults to 0)\n");
    fprintf(stderr, " -T\tTry to play future version.\n");
    fprintf(stderr, " -h\tdisplay this help message\n");
}

rosbag::PlayerOptions parseOptions(int argc, char** argv) {
    rosbag::PlayerOptions opts;

    int option_char;
    while ((option_char = getopt(argc, argv, "ncdahpb:r:s:t:q:T")) != -1) {
        switch (option_char) {
        case 'c': opts.check_bag    = true; break;
        case 'd': opts.show_defs    = true; break;
        case 'n': opts.quiet        = true; break;
        case 'a': opts.at_once      = true; break;
        case 'p': opts.start_paused = true; break;
        case 'T': opts.try_future   = true; break;
        case 'q': opts.queue_size         = atoi(optarg); break;
        case 'r': opts.time_scale         = atof(optarg); break;
        case 'b': opts.bag_time_frequency = atoi(optarg); opts.bag_time = true; break;
        case 's': opts.advertise_sleep    = (unsigned int) (1000000.0 * atof(optarg)); break;
        case 't': {
            char time[1024];
            strncpy(time, optarg, sizeof(time));
            opts.time = atof(time);

            opts.has_time = true;
            break;
        }
        //case 'h': printUsage(); ros::shutdown(); return;
        //case '?': printUsage(); ros::shutdown(); return;
        }
    }

    for (int i = optind; i < argc; i++)
        opts.bags.push_back(argv[i]);

    return opts;
}

int main(int argc, char** argv) {
    // Parse the command-line options
    rosbag::PlayerOptions opts;
    try {
        opts = parseOptions(argc, argv);
    }
    catch (ros::Exception const& ex) {
        fprintf(stderr, "Error reading options: %s", ex.what());
        return 1;
    }

    rosbag::Player player(opts);

    if (opts.check_bag) {
    	return player.checkBag();
    }
    else {
        ros::init(argc, argv, "play", ros::init_options::AnonymousName);
        try {
            player.publish();
        }
        catch (std::runtime_error& e) {
            ROS_FATAL("%s", e.what());
            return 1;
        }
    }
    
    return 0;
}
