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
#include <list>
#include <set>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <dirent.h>
#include "msgspec.h"
#include "utils.h"

using namespace std;

const char *fs_delim = "/"; // ifdef this for windows

extern set<string> &read_depfile( const char *filename, set<string> &deps );

class msg_gen
{
public:
  msg_gen() { }
  void process_file(const char *spec_file)
  {
    split_path(expand_path(spec_file), g_path, g_pkg, g_name);
    string cpp_dir = g_path + string("/lisp");
    string tgt_dir = g_path + string("/lisp/") + g_pkg;
    if (access(cpp_dir.c_str(), F_OK))
      if (mkdir(cpp_dir.c_str(), 0755) && (errno != EEXIST))
      {
        printf("woah! error from mkdir: [%s]\n", strerror(errno));
        exit(5);
      }

    if (access(tgt_dir.c_str(), F_OK) != 0)
      if (mkdir(tgt_dir.c_str(), 0755) && (errno != EEXIST))
      {
        printf("woah! error from mkdir: [%s]\n", strerror(errno));
        exit(5);
      }

    msg_spec spec(spec_file, g_pkg, g_name, g_path, true, true);
    char fname[PATH_MAX];

    // delete old dependencies file
    snprintf(fname, PATH_MAX, "%s/.%s.asd-dep", tgt_dir.c_str(), g_name.c_str() );
    remove( fname );

    snprintf(fname, PATH_MAX, "%s/%s.lisp", tgt_dir.c_str(), g_name.c_str());
    FILE *f = fopen(fname, "w");
    if (!f)
    {
      printf("woah! couldn't write to %s\n", fname);
      exit(7);
    }

    //fprintf(f, "(in-package %s)\n\n", g_pkg.c_str());

    /*
    string pkg_upcase = to_upper(g_pkg), msg_upcase = to_upper(g_name);
    fprintf(f, "#ifndef %s_%s_H\n",   pkg_upcase.c_str(), msg_upcase.c_str());
    fprintf(f, "#define %s_%s_H\n\n", pkg_upcase.c_str(), msg_upcase.c_str());
    fprintf(f, "#include <string>\n"
               "#include <vector>\n"
               "#include \"ros/message.h\"\n"
               "#include \"ros/time.h\"\n\n");
               */
    spec.emit_cpp_class(f);
    //fputs("#endif\n", f);
    fclose(f);

    // Also write the package.lisp file
    snprintf(fname, PATH_MAX, "%s/_package.lisp", tgt_dir.c_str());
    f = fopen(fname, "w");
    if (!f)
    {
      printf("woah! couldn't write to %s\n", fname);
      exit(7);
    }

    fprintf(f,"(defpackage %s-msg\n"
            "  (:use cl\n"
            "        roslisp-msg-protocol)\n"
            "  (:export\n",
            g_pkg.c_str());

    // Assemble a list of all the .msg files in the same directory, so that
    // the _package.lisp file has the complete list.  It's wasteful to
    // rewrite this file every time, but I don't see a reliable alternative
    // off-hand.
    DIR *d = opendir(g_path.c_str());
    if(!d)
    {
      printf("opendir error [%s] while crawling %s\n", 
             strerror(errno), g_path.c_str());
      exit(8);
    }
    struct dirent *ent;
    list<string> package_files;
    set<string> dependencies;
    while ((ent = readdir(d)) != NULL)
    {
      struct stat s;
      string fname = string(ent->d_name);
      string child_path = g_path + fs_delim + fname;
      if (stat(child_path.c_str(), &s) != 0) 
        continue;
      else if (S_ISDIR(s.st_mode)) 
        continue;
      else if (fname.size() < 5)
        continue;
      else if (fname.substr(fname.size()-4) != string(".msg"))
        continue;
      else
      {
        string path = fname.substr(0,fname.size()-4);
        string dep_file = g_path + string("/lisp/") + g_pkg + string("/.") + path + string(".asd-dep");

        read_depfile( dep_file.c_str(), dependencies );

        package_files.push_back(path);

        for(unsigned int i=0; i< path.size(); i++)
          path[i] = toupper(path[i]);
        fprintf(f,"   \"<%s>\"\n", path.c_str());
      }
    }
    fprintf(f,"  ))\n\n");

    fclose(f);

    // Emit asd file.
    snprintf(fname, PATH_MAX, "%s/%s-msg.asd", tgt_dir.c_str(), g_pkg.c_str());
    f = fopen(fname, "w");
    if (!f)
    {
      printf("woah! couldn't write to %s\n", fname);
      exit(7);
    }
    fprintf(f, "\n(in-package :asdf)\n\n");
    fprintf(f, "(defsystem \"%s-msg\"\n"
        "  :depends-on (:roslisp-msg-protocol :roslisp-utils", g_pkg.c_str());

    for(set<string>::iterator it = dependencies.begin();
        it != dependencies.end();
        ++it )
    {
        if( g_pkg + "-msg" != it->substr(0, g_pkg.size()+4) )
            fprintf(f, " :%s", it->c_str());
    }
    fprintf(f, ")\n");
    fprintf(f, "  :components ((:file \"_package\")\n    ");
    for(list<string>::iterator it=package_files.begin();
        it != package_files.end();
        ++it)
    {
        fprintf(f, "(:file \"%s\" :depends-on (\"_package\"))\n    "
            "(:file \"_package_%s\" :depends-on (\"_package\"))\n    ", it->c_str(), it->c_str());
    }
    fprintf(f, "))\n");
    
    fclose(f);
  }
};



int main(int argc, char **argv)
{
  if (argc <= 1)
  {
    printf("usage: genmsg_cpp MSG1 [MSG2] ...\n");
    return 1;
  }
  msg_gen gen;
  for (int i = 1; i < argc; i++)
    gen.process_file(argv[i]);
  return 0;
}
