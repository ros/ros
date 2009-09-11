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

#include <time.h>
#include <sys/stat.h>
#include "ros/node.h"
#include "rosrecord/Recorder.h"
#include "rosrecord/AnyMsg.h"
#include <string>

using namespace std;

class RosRecord
{

  typedef std::vector<std::pair<std::string, std::string> > TopicList;

public:

  ros::Node* node_;
  ros::record::Recorder recorder_;

  bool check_master_;
  bool verbose_;

  int exit_code_;

  int count_;

  char fname_[2048];
  char tgt_fname_[2048];

  void print_usage() {
    fprintf (stderr, "usage: rosrecord [-f filename] [TOPIC1 TOPIC2...]\n");
  }

  void print_help() {
    print_usage();
    fprintf(stderr, " -c num\tonly receive num messages on each topic\n");
    fprintf(stderr, " -f prefix\tappend file prefix to beginning of name (name will always end with date stamp)\n");
    fprintf(stderr, " -v\tdisplay a message every time a message is received on a topic\n");
    fprintf(stderr, " -h\tdisplay this help message\n");
  }

  RosRecord(int argc, char** argv) : node_(new ros::Node("rosrecord", ros::Node::ANONYMOUS_NAME)), recorder_(node_), check_master_(false), verbose_(false), exit_code_(0), count_(-1)
  {
    char name[1024];
    name[0] = 0;
    char prefix[1024];
    prefix[0] = 0;
    time_t t = ::time(NULL);
    struct tm *tms = localtime(&t);
    snprintf(name, sizeof(name), "%d-%02d-%02d-%02d-%02d-%02d-topic.bag",
             tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
             tms->tm_hour     , tms->tm_min  , tms->tm_sec);

    int option_char;

    while ((option_char = getopt(argc,argv,"f:c:hv")) != -1)
      switch (option_char)
      {
      case 'f': strncpy(prefix,optarg,sizeof(prefix)); break;
      case 'c': count_ = atoi(optarg); break;
      case 'v': verbose_ = true; break;
      case 'h': print_help(); node_->shutdown(); return;
      case '?': print_usage(); node_->shutdown(); return;
      }

    if (strlen(prefix) == 0)
    {
      strcpy(tgt_fname_, name);
    } else {
      snprintf(tgt_fname_, sizeof(tgt_fname_), "%s_%s", prefix, name);
    }
    snprintf(fname_, sizeof(fname_), "%s.active", tgt_fname_);

    if ((argc - optind) < 1)
    {
      if (count_ > 0)
      {
        fprintf(stderr, "Specifing a count is not valid with automatic topic subscription.\n");
        exit_code_ = 1;
        node_->shutdown();
        return;
      }

      check_master_ = true;
    }

    vector<std::string> topics;
    for (int i = optind; i < argc; i++)
      topics.push_back(argv[i]);

    ros::Time start = ros::Time(0,0);

    if (recorder_.open(std::string(fname_), start))
    {
      for (vector<std::string>::iterator i = topics.begin(); i != topics.end(); i++)
      {
        int* count = new int;
        *count = count_;

        recorder_.addTopic<AnyMsg>(*i, 100, &RosRecord::msg_handler, this, count);
      }

      checkMaster();

      recorder_.start();
    } else {
      exit_code_ = 1;
      node_->shutdown();
    }
  }

  ~RosRecord()
  {
    recorder_.close();
    rename(fname_,tgt_fname_);

    delete node_;
  }

  void checkMaster() {
    if (check_master_)
    {
      TopicList topics;

      node_->getPublishedTopics(&topics);

      for (TopicList::iterator i = topics.begin(); i != topics.end(); i++)
        recorder_.addTopic<AnyMsg>(i->first, 100, &RosRecord::msg_handler, this, NULL);
    }
  }

  void msg_handler(std::string name, ros::Message* msg, ros::Time t, ros::Time t_not_use, void* p)
  {
    if (verbose_)
      std::cout << "Received message on topic " << name << std::endl;

    if (p != NULL)
    {
      int* count = (int*)(p);
      if ((*count) > 0)
      {
        (*count)--;
        if (*count == 0)
        {
          node_->unsubscribe(name);
          delete count;
        }
      }
    }
  }

  bool spin() {
    while (node_->ok())
    {
      checkMaster();
      if (node_->numSubscriptions() == 0)
        node_->shutdown();
      usleep(1000000);
    }
    return true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  RosRecord rl(argc, argv);

  rl.spin();


  return rl.exit_code_;
}
