/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Morgan Quigley */

// this little program just snarfs up the master URI from the environment,
// splits it into a host:port. then sends it a 'shutdown' xml-rpc request

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include "XmlRpc.h"
using namespace XmlRpc;

int main(int, char **)
{
  char *env_master_uri = getenv("ROS_MASTER_URI");
  if (!env_master_uri)
  {
    printf("woah! no ROS_MASTER_URI in the environment. umm....\n");
    return 1;
  }
  char master_uri[500];
  strncpy(master_uri, env_master_uri, sizeof(master_uri));
  char *colon_pos = strrchr(master_uri, ':');
  if (!colon_pos)
  {
    printf("malformed master uri\n");
    return 1;
  }
  *colon_pos = ' ';
  char host_str[450], port_str[50];
  // skip over the http:// when telling xmlrpc++ the hostname
  if (2 != sscanf(master_uri+7, "%450s %50s", host_str, port_str))
  {
    printf("malformed master uri\n");
    return 1;
  }
  int port = atoi(port_str);
  if (!port)
  {
    printf("malformed master uri\n");
    return 1;
  }

  printf("shutting down the master at %s on port %d\n", host_str, port);
  XmlRpcClient c(host_str, port);
  XmlRpcValue args, result;
  args[0] = "testerbot";
  try
  {
    if (c.execute("shutdown", args, result))
      printf("shutdown request OK.\n");
    else
      printf("error trying to send shutdown request.\n");
  }
  catch (...)
  {
    printf("exception trying to send shutdown request.\n");
  }

  // todo: to make this a fully automated unit test, I need to grab the PID
  // of botherder/zenmaster and ensure that it goes away after a few ms
  return 0;
}

