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

#include <cstdio>
#include "utils.h"
#include <sys/param.h>
#include <cstdlib>

using namespace std;

string g_path, g_pkg, g_ns_name, g_name;

void string_split(const string &s, vector<string> &t, const string &d)
{
  t.clear();
  size_t start = 0, end;
  while ((end = s.find_first_of(d, start)) != string::npos)
  {
    t.push_back(s.substr(start, end-start));
    start = end + 1;
  }
  t.push_back(s.substr(start));
}

string string_join(const vector<string> &t, const string &d)
{
  if (t.size() == 0)
    return string();
  else if (t.size() == 1)
    return d;
  string s;
  size_t i;
  for (i = 0; i < t.size()-1; i++)
    s += t[i] + d;
  return s + t[i];
}

string to_upper(const string &s)
{
  string t;
  for (size_t i = 0; i < s.length(); i++)
    t += toupper(s[i]);
  return t;
}

void split_path(const string &full_path, string &msg_path, 
                string &msg_pkg, string &msg_name)
{
  vector<string> tokens;
  string_split(full_path, tokens, string("/"));
  if (tokens.size() < 3)
  {
    printf("woah! error in split_path. not enough tokens. very bad.\n");
    exit(3);
  }
  msg_path = string_join(tokens, string("/"));
  msg_name = tokens[tokens.size()-1];
  msg_name = msg_name.substr(0, msg_name.length() - 4);
  msg_pkg = tokens[tokens.size()-3];
  tokens.erase(tokens.end()-1);
  msg_path = string_join(tokens, string("/"));
}

string expand_path(const string &path)
{
  if (path[0] == '/')
    return path;
  else
  {
    char resolved[PATH_MAX];
    if(!realpath(path.c_str(), resolved))
    {
      perror("realpath() failed");
      exit(4);
    }
    return string(resolved);
  }
}

bool rospack_check_dep(const string &pkg, const string& dep)
{
  if((dep == pkg) || (dep == string("roslib")))
    return true;
  string cmd = string("rospack deps1 ") + pkg;
  FILE *rospipe = popen(cmd.c_str(), "r");
  if (!rospipe)
  {
    printf("couldn't launch rospack.\n");
    exit(10);
  }
  bool found = false;
  char rosout[PATH_MAX];
  while(fgets(rosout, PATH_MAX, rospipe))
  {
    string ret(rosout);
    for (size_t newline = ret.find('\n'); newline != string::npos; 
                newline = ret.find('\n'))
      ret.erase(newline, 1);
    if(ret == dep)
    {
      found = true;
      break;
    }
  }
  return found;
}

string rospack_find(const string &pkg)
{
  string cmd = string("rospack find ") + pkg;
  FILE *rospipe = popen(cmd.c_str(), "r");
  if (!rospipe)
  {
    printf("couldn't launch rospack.\n");
    exit(10);
  }
  char rosout[PATH_MAX];
  if (fgets(rosout, PATH_MAX, rospipe))
  {
    string ret(rosout);
    for (size_t newline = ret.find('\n'); newline != string::npos; 
                newline = ret.find('\n'))
      ret.erase(newline, 1);
    pclose(rospipe);
    return ret;
  }
  printf("error in rospack read\n");
  pclose(rospipe);
  exit(11);
  return string(); // not really
}
