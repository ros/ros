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

#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "utils.h"

using std::string;
using std::vector;

bool is_primitive(const string &type)
{
  vector<string> prims;
  prims.push_back("byte");
  prims.push_back("char");
  prims.push_back("uint8");
  prims.push_back("int8");
  prims.push_back("uint16");
  prims.push_back("int16");
  prims.push_back("uint32");
  prims.push_back("int32");
  prims.push_back("uint64");
  prims.push_back("int64");
  prims.push_back("float32");
  prims.push_back("float64");
  prims.push_back("string");
  prims.push_back("time");
  prims.push_back("duration");
  for (vector<string>::iterator i = prims.begin(); i != prims.end(); ++i)
    if (*i == type)
      return true;
  return false;
}

vector<string> find_deps(const string &msgfile, vector<string> deps)
{
  string msg_path, msg_pkg, msg_name;
  split_path(expand_path(msgfile), msg_path, msg_pkg, msg_name);
  FILE *f = fopen(msgfile.c_str(), "r");
  if (!f)
  {
    printf("couldn't open [%s]\n", msgfile.c_str());
    exit(1);
  }
  const int LINEBUF_LEN = 1024;
  char linebuf[LINEBUF_LEN];
  for (int linenum = 1; !feof(f); linenum++)
  {
    if (!fgets(linebuf, LINEBUF_LEN, f))
      break; // hit eof
    if (strlen(linebuf) == 0)
      continue;
    size_t i;
    for (i = 0; i < strlen(linebuf); i++)
      if (linebuf[i] != ' ' && linebuf[i] != '\t')
        break; // go find first non-whitespace character
    if (i == strlen(linebuf) || linebuf[i] == '\n')
      continue; // blank line.
    if (linebuf[i] == '#')
      continue; // comment.
    if (linebuf[i] == '-')
      continue; // message separator in services
    char *token = strtok(linebuf, " [\t\n");
    if (!token)
    {
      printf("couldn't parse a token out of spec file %s on line %d\n",
             msgfile.c_str(), linenum);
      exit(2);
    }
    string type(token);
    if (is_primitive(type))
      continue;

    vector<string> type_vec;
    string_split(type, type_vec, "/");
    string spec_name, package;
    if (type_vec.size() >= 2)
    {
      package = type_vec[0];
      spec_name = type_vec[1];
    }
    else
    {
      spec_name = type_vec[0];
      package = msg_pkg;
    }
    if (spec_name == "Header")
      package = "roslib";
    string pkg_path = rospack_find(package);
    if (pkg_path.length() == 0)
    {
      printf("Error: couldn't find package %s\n", package.c_str());
      exit(13);
    }
    string spec_file = pkg_path + string("/msg/") + spec_name + string(".msg");
    deps = find_deps(spec_file, deps);
    bool found = false;
    for (size_t i = 0; !found && i < deps.size(); i++)
      if (deps[i] == spec_file)
        found = true;
    if (!found)
      deps.push_back(spec_file);
  }
  fclose(f);
  return deps;
}

int main(int argc, char **argv)
{
  if (argc <= 1)
  {
    printf("usage: submsgs MSGFILE\n");
    return 1;
  }
  string msgfile(argv[1]);
  vector<string> deps = find_deps(msgfile, vector<string>());
  for (size_t i = 0; i < deps.size(); i++)
    printf("%s\n", deps[i].c_str());
  return 0;
}

