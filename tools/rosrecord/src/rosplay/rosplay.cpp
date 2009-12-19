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

#include "rosrecord/rosplay.h"

#include "sys/select.h"

using namespace std;
using namespace ros;


void print_usage() 
{
  fprintf (stderr, "Usage: rosplay [options] BAG1 [BAG2]\n");
}


void print_help() 
{
  print_usage();
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




RosPlay::RosPlay(int i_argc, char **i_argv) : 
  node_handle(NULL),
  bag_time_initialized_(false),
  at_once_(false), 
  quiet_(false),
  paused_(false), 
  shifted_(false), 
  bag_time_(false),
  time_scale_(1.0),
  queue_size_(0),
  bag_time_publisher_(NULL)
{
  const int fd = fileno(stdin);
  
  advertise_sleep_ = 200000;
  
  termios flags;
  tcgetattr(fd,&orig_flags_);
  flags = orig_flags_;
  flags.c_lflag &= ~ICANON;  // set raw (unset canonical modes)
  flags.c_cc[VMIN]  = 0;     // i.e. min 1 char for blocking, 0 chars for non-blocking
  flags.c_cc[VTIME] = 0;     // block if waiting for char
  tcsetattr(fd,TCSANOW,&flags);

  FD_ZERO(&stdin_fdset_);
  FD_SET(fd, &stdin_fdset_);
  maxfd_ = fd + 1;
  
  char time[1024];
  bool has_time = false;  
  int option_char;
  
  bool try_future = false;

  while ((option_char = getopt(i_argc,i_argv,"ncahpb:r:s:t:q:T")) != -1){
    switch (option_char){
    case 'c': ros::shutdown(); break;  // This shouldn't happen
    case 'n': quiet_ = true; break;
    case 'a': at_once_ = true; break;
    case 'p': paused_ = true; break;
    case 's': advertise_sleep_ = (unsigned int)(1000000.0*atof(optarg)); break;
    case 't': strncpy(time,optarg,sizeof(time)); has_time=true; break;
    case 'q': queue_size_ = atoi(optarg); break;
    case 'b': bag_time_frequency_ =  atoi(optarg); bag_time_ = true; break;
    case 'r': time_scale_ =  atof(optarg); break;
    case 'T': try_future = true; break;
    case 'h': print_help(); ros::shutdown(); return;
    case '?': print_usage(); ros::shutdown(); return;
    }
  }
  //std::cout << "check options" << std::endl;
  
  float float_time = 0.0f;
  if (has_time)
    float_time = atof(time);

  if (optind == i_argc){
    fprintf(stderr, "You must specify at least one bagfile to play from.\n");
    print_help(); ros::shutdown(); return;
  }
  
  std::vector<std::string> bags;
  for (int i = optind; i < i_argc; i++) 
    bags.push_back(i_argv[i]);


  if (bag_time_ && bags.size() > 1){
    fprintf(stderr, "You can only play one single bag when using bag time [-b].\n");
    print_usage(); ros::shutdown(); return;
  }

  node_handle = new ros::NodeHandle;
  bag_time_publisher_ = new TimePublisher;
  
  if (bag_time_)
    bag_time_publisher_->initialize(bag_time_frequency_, time_scale_);

  start_time_ = getSysTime();
  requested_start_time_ = start_time_;
  
  if (player_.open(bags, start_time_ + ros::Duration().fromSec(-float_time), time_scale_, try_future))
    player_.addHandler<AnyMsg>(string("*"), &RosPlay::doPublish, this, NULL, false);
  else
  {
    // Not sure exactly why, but this shutdown is necessary for the
    // exception to be received by the main thread without giving a Ctrl-C.
    ros::shutdown();  
    throw std::runtime_error("Failed to open one of the bag files.");
  }

  if (!at_once_){
    if (paused_){
      paused_time_ = getSysTime();
      std::cout << "Hit space to resume, or 's' to step.";
      std::cout.flush();
    } else {
      std::cout << "Hit space to pause.";
      std::cout.flush();
    }
  }
}



RosPlay::~RosPlay() 
{
  if (node_handle)
  {
    // This sleep shouldn't be necessary
    usleep(1000000);
    delete node_handle;
  }

  if (bag_time_publisher_)
    delete bag_time_publisher_;

  // Fix terminal settings.
  const int fd = fileno(stdin);
  tcsetattr(fd,TCSANOW,&orig_flags_);
}



bool RosPlay::spin() 
{
  if (node_handle && node_handle->ok()){
    if(!quiet_)
      puts("");
    ros::WallTime last_print_time(0.0);
    ros::WallDuration max_print_interval(0.1);
    while (node_handle->ok()){
      if (!player_.nextMsg())
	break;
      ros::WallTime t = ros::WallTime::now();
      if(!quiet_ && ((t - last_print_time) >= max_print_interval))
      {
        printf("Time: %16.6f    Duration: %16.6f\r",
               ros::Time::now().toSec(), player_.getDuration().toSec());
        fflush(stdout);
        last_print_time = t;
      }
    }
    std::cout << std::endl << "Done." << std::endl;
    // Request a shutdown
    ros::shutdown();
  }

  return true;
}



Time RosPlay::getSysTime()
{
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  return Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);
}

std::map<std::string, ros::Publisher> g_publishers;

void RosPlay::doPublish(string name, ros::Message* m, ros::Time play_time, ros::Time record_time, void* n)
{
  if (play_time < requested_start_time_)
    return;
  
  // if we are using the bag time
  if (bag_time_){
    // initialize bag time publisher
    if (!bag_time_initialized_){
      // starting in paused mode
      if (paused_)
	bag_time_publisher_->stepTime(record_time);
      // starting in play mode
      else 
	bag_time_publisher_->startTime(record_time);
      bag_time_initialized_ = true;
    }
    // at once
    if (at_once_)
      bag_time_publisher_->startTime(record_time);
    else
      bag_time_publisher_->setHorizon(play_time);
  }
  

  
  std::map<std::string, ros::Publisher>::iterator pub_token = g_publishers.find(name);

  // advertise the topic to publish
  //  if (ros::Node::instance()->advertise(name, *m, queue_size_))
  if (pub_token == g_publishers.end())
  {
    AdvertiseOptions opts(name, queue_size_, m->__getMD5Sum(), m->__getDataType(), m->__getMessageDefinition());
    ros::Publisher pub = node_handle->advertise(opts);
    g_publishers.insert(g_publishers.begin(), std::pair<std::string, ros::Publisher>(name, pub));
    pub_token = g_publishers.find(name);

    if (bag_time_) bag_time_publisher_->freezeTime();
    Time paused_time_ = getSysTime();
    ROS_INFO("Sleeping %.3f seconds after advertising %s...",
             advertise_sleep_ / 1e6, name.c_str());
    usleep(advertise_sleep_);
    ROS_INFO("Done sleeping.\n");
    Duration shift = getSysTime() - paused_time_;
    player_.shiftTime(shift);
    if (bag_time_) bag_time_publisher_->startTime(record_time);
  }
  
  if (!at_once_){

    ros::Time now = getSysTime();
    ros::Duration delta = play_time - getSysTime();
    
    while ( (paused_ || delta > ros::Duration(0,100000)) && node_handle->ok()){
      bool charsleftorpaused = true;
      
      while (charsleftorpaused && node_handle->ok()){
	//Read from stdin:
        
        char c = EOF;

        #ifdef __APPLE__

        fd_set testfd;
        FD_COPY(&stdin_fdset_, &testfd);

        #else

        fd_set testfd = stdin_fdset_;

        #endif

        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        if (select(maxfd_, &testfd, NULL, NULL, &tv) > 0)
          c = getc(stdin);

	switch (c){
	case ' ':
	  paused_ = !paused_;
	  if (paused_) {
	    if (bag_time_) bag_time_publisher_->freezeTime();
	    paused_time_ = getSysTime();
	    std::cout << std::endl << "Hit space to resume, or 's' to step.";
	    std::cout.flush();
	  } else {
	    if (bag_time_) bag_time_publisher_->startTime(record_time);
	    ros::Duration shift;
	    if (shifted_){
	      shift = getSysTime() - play_time;
	      play_time = getSysTime();
	      shifted_ = false;
	    } else {
	      shift = getSysTime() - paused_time_;
	      play_time = play_time + shift;
	    }
	    player_.shiftTime(shift);
	    std::cout << std::endl << "Hit space to pause.";
	    std::cout.flush();
	  }
	  break;
	case 's':
	  if (paused_){
	    shifted_ = true;
	    if (bag_time_) bag_time_publisher_->stepTime(record_time);
            (pub_token->second).publish(*m);
	    return;
	  }
	  break;
	case EOF:
	  if (paused_)
	    usleep(10000);
	  else
	    charsleftorpaused = false;
	}
      }
      
      usleep(100000);
      
      delta = play_time - getSysTime();
    }
    
    if (!paused_ && delta > ros::Duration(0, 5000) && node_handle->ok())
      usleep(delta.toNSec()/1000 - 5); // Should this be a ros::Duration::Sleep?
  }
  (pub_token->second).publish(*m);
}






struct BagContent
{
  string datatype;
  string md5sum;
  string definition;
  int count;
  BagContent(string d, string m, string def) : datatype(d), md5sum(m), definition(def), count(1) {}
};


map<string, BagContent> g_content;
unsigned long long g_end_time;




void checkFile(string name, ros::Message* m, ros::Time time_play, ros::Time time_recorded, void* n)
{
  map<string, BagContent>::iterator i = g_content.find(name);

  if (i == g_content.end())
  {
    g_content.insert(pair<string, BagContent>(name, BagContent(m->__getDataType(), m->__getMD5Sum(), m->__getMessageDefinition())));
  } else {
    i->second.count++;
  }
  g_end_time = time_play.toNSec();
}




int main(int argc, char **argv)
{

  bool check_bag = false; // We intercept this before starting the node since checkbag doesn't require a node
  bool show_defs = false;

  for (int i = 0; i < argc; i++)
  {
    if (!strcmp(argv[i],"-c") || !strcmp(argv[i],"-cd"))
    {
      check_bag = true;
    }
  }

  if (check_bag)
  {
    bool try_future = false;

    int option_char;
    while ((option_char = getopt(argc,argv,"cdahpt:q:T")) != -1)
      switch (option_char)
      {
      case 'c': break;
      case 'd': show_defs = true; break;
      case 'a': fprintf(stderr, "Option -a is not valid when checking bag\n"); return 1;
      case 'p': fprintf(stderr, "Option -p is not valid when checking bag\n"); return 1;
      case 't': fprintf(stderr, "Option -t is not valid when checking bag\n"); return 1;
      case 'q': fprintf(stderr, "Option -q is not valid when checking bag\n"); return 1;
      case 'T': try_future = true; break;
      case 'h': print_help();  return 0;
      case '?': print_usage();  return 0;
      }

    if (argc - optind > 1)
    {
      fprintf(stderr, "Only 1 bag can be checked at a time\n");
      return 1;
    }

    ros::record::Player player;

    if (player.open(argv[optind], ros::Time(), try_future))
    {
      player.addHandler<AnyMsg>(string("*"), &checkFile, NULL, false);
    }

    while(player.nextMsg())
    {
    }

    printf("bag: %s\n", argv[optind]);
    printf("version: %s\n", player.getVersionString().c_str());
    printf("start_time: %lld\n", (long long int)player.getFirstDuration().toNSec());
    printf("end_time: %lld\n", g_end_time + player.getFirstDuration().toNSec());
    printf("length: %lld\n", g_end_time);
    printf("topics:\n");

    for (map<string, BagContent>::iterator i = g_content.begin();
         i != g_content.end();
         i++)
    {
      printf("  - name: %s\n", i->first.c_str());
      printf("    count: %d\n", i->second.count);
      printf("    datatype: %s\n", i->second.datatype.c_str());
      printf("    md5sum: %s\n", i->second.md5sum.c_str());

      if (show_defs)
      {
        string def = i->second.definition.c_str();

        if (def.length() > 0)
        {
          printf("    definition: |\n");
      
          size_t oldind = 0;
          size_t ind = def.find_first_of('\n', 0);

          while (ind != def.npos)
          {
            printf("      %s\n", def.substr(oldind, ind - oldind).c_str());
            oldind = ind + 1;
            ind = def.find_first_of('\n', oldind);
          }
      
          ind = def.length();
      
          printf("      %s\n", def.substr(oldind, ind - oldind).c_str());
        } else {
          printf("    definition: NONE\n");
        }
      }

    }
    return 0;
  } // if check bag

  ros::init(argc, argv, "rosplay", ros::init_options::AnonymousName);

  try
  {
    RosPlay rp(argc, argv);
    rp.spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_FATAL("%s", e.what());
    return 1;
  }

  return 0;
}




