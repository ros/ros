///////////////////////////////////////////////////////////////////////////////
// The roscpp package provides a c++ client library implementation for ROS.
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#ifndef TOPIC_H
#define TOPIC_H

#include <string>
#include <vector>
using namespace std;

namespace ros
{

class process
{
public:
  string name, uri;
  process(string _name, string _uri) : name(_name), uri(_uri) { }
  bool operator==(const process &other) const
  {
    return name == other.name && uri == other.uri;
  }
};

class topic
{
public:
  string name, datatype;
  vector<process *> pubs, subs;
  topic(string _name, string _datatype) : name(_name), datatype(_datatype) { }
  bool add_pub(process *p);
  bool add_sub(process *p);
  bool remove_pub(process *p);
  bool remove_sub(process *p);
  vector<string> pub_uris();
  vector<string> sub_uris();
  void unregister_publisher(string pub_name);
  int  num_procs();
  bool has_pub(process *p);
  bool has_sub(process *p);
  /** stuffs an update-request message into the send queue */
  void send_publisher_updates_async();
  /** the worker thread function for sending publisher updates. */
  /*
  static void send_publisher_updates_worker(string _name,
                                            vector<string> sub_uris_vec,
                                            vector<string> pub_uris_vec);
  */
};

}

#endif

