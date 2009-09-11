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

#include <sstream>
#include "topic.h"
#include "master.h"

using namespace ros;

bool topic::add_pub(process *p)
{
  for (vector<process *>::iterator i = pubs.begin(); i != pubs.end(); ++i)
    if (**i == *p)
      return false;
  pubs.push_back(p);
  return true;
}

bool topic::add_sub(process *p)
{
  for (vector<process *>::iterator i = subs.begin(); i != subs.end(); ++i)
    if (**i == *p)
      return false;
  subs.push_back(p);
  return true;
}

bool topic::remove_pub(process *p)
{
  for (vector<process *>::iterator i = pubs.begin(); i != pubs.end(); ++i)
    if (**i == *p)
    {
      pubs.erase(i);
      return true;
    }
  return false; // never found it.
}

bool topic::remove_sub(process *p)
{
  for (vector<process *>::iterator i = subs.begin(); i != subs.end(); ++i)
    if (**i == *p)
    {
      subs.erase(i);
      return true;
    }
  return false; // never found it.
}

vector<string> topic::pub_uris()
{
  vector<string> uris;
  for (vector<process *>::iterator i = pubs.begin(); i != pubs.end(); ++i)
    uris.push_back((*i)->uri);
  return uris;
}

vector<string> topic::sub_uris()
{
  vector<string> uris;
  for (vector<process *>::iterator i = subs.begin(); i != subs.end(); ++i)
    uris.push_back((*i)->uri);
  return uris;
}

void topic::unregister_publisher(string pub_name)
{
  for (vector<process *>::iterator i = pubs.begin(); i != pubs.end(); ++i)
    if ((*i)->name == pub_name)
    {
      pubs.erase(i);
      return;
    }
}

int topic::num_procs()
{
  return pubs.size() + subs.size();
}

bool topic::has_pub(process *p)
{
  for (vector<process *>::iterator i = pubs.begin(); i != pubs.end(); ++i)
    if (*i == p)
      return true;
  return false;
}

bool topic::has_sub(process *p)
{
  for (vector<process *>::iterator i = subs.begin(); i != subs.end(); ++i)
    if (*i == p)
      return true;
  return false;
}


/*
struct pub_update_params
{
  string name;
  vector<string> sub_uris_vec;
  vector<string> pub_uris_vec;
};
*/

void topic::send_publisher_updates_async()
{
  pub_update pu;
  pu.name = name;
  pu.sub_uris_vec = sub_uris();
  pu.pub_uris_vec = pub_uris();
  g_master->enqueue_pub_update(pu);
  
/*
  pub_update_params *p = new pub_update_params;
  p->name = name;
  p->sub_uris_vec = sub_uris();
  p->pub_uris_vec = pub_uris();
  pthread_t dummy;
  pthread_create(&dummy, NULL, s_publisher_updates_entry, p);
  pthread_detach(dummy); // see ya
*/
}

