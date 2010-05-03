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

#include "rosrecord/Recorder.h"

#include "rosrecord/constants.h"

#include <iomanip>
#include <signal.h>
#include <sys/statvfs.h>

#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/bzip2.hpp>

using std::string;

namespace ros {
namespace record {

Recorder::Recorder() :
    logging_enabled_(true)
{
}

Recorder::~Recorder()
{
    close();
}

bool Recorder::open(string const& file_name, bool random_access)
{
    try
    {
        bag_.open(file_name, rosbag::bagmode::Write);

        check_disk_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(20.0);
        warn_next_ = ros::WallTime();
        checkDisk();

        return true;
    }
    catch (rosbag::BagException ex)
    {
        ROS_FATAL("rosrecord::Record: Failed to open file: %s", file_name.c_str());
        return false;
    }
}

pos_t Recorder::getOffset() { return bag_.getSize(); }

void Recorder::close()
{
    bag_.close();
}

bool Recorder::checkDisk()
{
    struct statvfs fiData;

    string filename = bag_.getFileName();

    if ((statvfs(filename.c_str(), &fiData)) < 0)
    {
        ROS_WARN("rosrecord::Record: Failed to check filesystem stats.");
    }
    else
    {
        unsigned long long free_space = 0;

        free_space = (unsigned long long)(fiData.f_bsize) * (unsigned long long)(fiData.f_bavail);

        if (free_space < 1073741824ull)
        {
            ROS_ERROR("rosrecord::Record: Less than 1GB of space free on disk with %s.  Disabling logging.", filename.c_str());
            logging_enabled_ = false;
            return false;
        }
        else if (free_space < 5368709120ull)
        {
            ROS_WARN("rosrecord::Record: Less than 5GB of space free on disk with %s.", filename.c_str());
        }
        else
        {
            logging_enabled_ = true;
        }
    }
    return true;
}

bool Recorder::record(string topic_name, ros::Message::ConstPtr msg, ros::Time time)
{
    return record(topic_name, *msg, time);
}

bool Recorder::record(string topic_name, const ros::Message& msg, ros::Time time)
{
    if (!logging_enabled_)
    {
        ros::WallTime nowtime = ros::WallTime::now();
        if (nowtime > warn_next_)
        {
            warn_next_ = nowtime + ros::WallDuration().fromSec(5.0);
            ROS_WARN("Not logging message because logging disabled.  Most likely cause is a full disk.");
        }
        return false;
    }

    {
        boost::mutex::scoped_lock lock(check_disk_mutex_);

        if (ros::WallTime::now() > check_disk_next_)
        {
            check_disk_next_ = check_disk_next_ + ros::WallDuration().fromSec(20.0);

            if (!checkDisk())
                return false;
        }
    }

    boost::shared_ptr<ros::M_string> hdr = msg.__connection_header;
    if (hdr == NULL)
        hdr = boost::shared_ptr<ros::M_string>(new ros::M_string);
    (*hdr)["type"]               = std::string(ros::message_traits::datatype(msg));
    (*hdr)["md5sum"]             = std::string(ros::message_traits::md5sum(msg));
    (*hdr)["message_definition"] = std::string(ros::message_traits::definition(msg));
    if (hdr->find("callerid") == hdr->end())
        (*hdr)["callerid"] = string("");
    if (hdr->find("latching") == hdr->end())
        (*hdr)["latching"] = string("0");

    try
    {
        bag_.write(topic_name, time, msg, hdr);
    }
    catch (rosbag::BagException ex)
    {
        ROS_FATAL("rosrecord::Record: could not write to file.  Check permissions and diskspace\n");
        return false;
    }

    return true;
}

} // namespace record
} // namespace ros
