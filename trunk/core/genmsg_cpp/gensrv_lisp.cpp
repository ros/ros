///////////////////////////////////////////////////////////////////////////////
// The roscpp package provides a c++ implementation for ROS.
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

#include <cstdio>
#include <list>
#include <set>
#include <stdexcept>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <dirent.h>
#include <sstream>
#include "utils.h"
#include "msgspec.h"
using namespace std;

class srv_gen;
srv_gen *g_srv_gen = NULL;

const char *fs_delim = "/"; // ifdef this for windows

class srv_gen
{
  string md5sum;
public:
  srv_gen() { g_srv_gen = this; }
  void process_file(const char *spec_file)
  {
    // compute md5sum
    string cmd = string("`rospack find roslib`/scripts/gendeps --md5 ") + spec_file;
    FILE *md5pipe = popen(cmd.c_str(), "r");
    if (!md5pipe)
      throw std::runtime_error("couldn't launch md5sum in genmsg_cpp\n");
    char md5buf[PATH_MAX];
    if (!fgets(md5buf, PATH_MAX, md5pipe))
      throw std::runtime_error("couldn't read md5sum pipe in genmsg_cpp\n");
    char *md5str = strtok(md5buf, " \t\n");
    md5sum = string(md5str);
    // call pclose sometime
    split_path(expand_path(spec_file), g_path, g_pkg, g_name);
    string lisp_dir = g_path + string("/lisp");
    string tgt_dir = g_path + string("/lisp/") + g_pkg;
    if (access(lisp_dir.c_str(), F_OK))
      if (mkdir(lisp_dir.c_str(), 0755) && (errno != EEXIST))
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

    FILE *in_f = fopen(spec_file, "r");
    if (!in_f)
    {
      printf("couldn't open service spec file [%s]\n", spec_file);
      exit(20);
    }
    string request, response;
    const int LINEBUF_LEN = 1024;
    char linebuf[LINEBUF_LEN];
    bool in_request = true;
    for (int linenum = 1; !feof(in_f); linenum++)
    {
      if (!fgets(linebuf, LINEBUF_LEN, in_f))
        break; // hit EOF
      if (!strncmp(linebuf, "---", 3))
      {
        in_request = false;
        continue; // skip the rest of this line.
      }
      if (in_request)
        request += linebuf;
      else
        response += linebuf;
    }
    fclose(in_f);
    msg_spec request_spec(request, g_pkg, g_name, g_path, false, true);
    msg_spec response_spec(response, g_pkg, g_name, g_path, false, true);
    request_spec.server_md5sum = md5sum;
    response_spec.server_md5sum = md5sum;
    request_spec.service_datatype = g_pkg + "/" + g_name;
    response_spec.service_datatype = g_pkg + "/" + g_name;

    {
      std::stringstream ss;
      // compute concatenated definition
      string cmd = string("`rospack find roslib`/scripts/gendeps --cat ") + spec_file;
      FILE *catpipe = popen(cmd.c_str(), "r");
      if (!catpipe)
        throw std::runtime_error("couldn't launch gendeps in genmsg_cpp\n");
      string request_def, response_def;
      bool in_request = true;
      for (int linenum = 1; !feof(catpipe); linenum++)
      {
        if (!fgets(linebuf, LINEBUF_LEN, catpipe))
          break; // hit EOF
        if (!strncmp(linebuf, "---", 3))
        {
          in_request = false;
          continue; // skip the rest of this line.
        }
        if (in_request)
          request_def += linebuf;
        else
          response_def += linebuf;
      }
      // call pclose sometime

      request_spec.full_definition = request_def;
      response_spec.full_definition = response_def;
    }
    
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
    string pkg_upcase = to_upper(g_pkg), srv_upcase = to_upper(g_name);
    //fprintf(f, "#ifndef SRV_%s_%s_H\n", pkg_upcase.c_str(), srv_upcase.c_str());
    //fprintf(f, "#define SRV_%s_%s_H\n", pkg_upcase.c_str(), srv_upcase.c_str());
    //fprintf(f, "\n#include <string>\n#include <vector>\n\n");
    request_spec.emit_cpp_includes(f);
    response_spec.emit_cpp_includes(f);
    fprintf(f, "\n");
    fprintf(f, "(in-package %s-srv)\n\n", g_pkg.c_str());
    fprintf(f, "\n");
    //fprintf(f, "namespace %s\n{\n\n", g_pkg.c_str());
    //fprintf(f, "namespace %s\n{\n\n", g_name.c_str());
    string srv_name = g_name;
    g_name = srv_name + "-request";
    request_spec.emit_cpp_class(f, true, srv_name + "Request");
    g_name = srv_name + "-response";
    response_spec.emit_cpp_class(f, true, srv_name + "Response");
    g_name = srv_name;
    fprintf(f, "(defmethod service-request-type ((msg (eql '%s)))\n"
               "  '<%s-request>)\n", srv_name.c_str(), srv_name.c_str());
    fprintf(f, "(defmethod service-response-type ((msg (eql '%s)))\n"
               "  '<%s-response>)\n", srv_name.c_str(), srv_name.c_str());
    fprintf(f, "(defmethod ros-datatype ((msg (eql '%s)))\n"
            "  \"Returns string type for a service object of type '<%s>\"\n"
            "  \"%s/%s\")\n",
            srv_name.c_str(),
            srv_name.c_str(), 
            g_pkg.c_str(), srv_name.c_str());
    /*
    fprintf(f, "class %s : public ros::service\n{\npublic:\n",
            g_name.c_str());
    fprintf(f, "  %s::%s_request request;\n"
               "  %s::%s_response response;\n",
            g_pkg.c_str(), g_name.c_str(), g_pkg.c_str(), g_name.c_str());
    fprintf(f, "  %s() : ros::service(\"%s/%s\", \"%s\")\n  {\n  }\n",
            g_name.c_str(), g_pkg.c_str(), g_name.c_str(), md5sum.c_str());
    fprintf(f, "  virtual ~%s() { }\n", g_name.c_str());
    fprintf(f, "  static std::string get_datatype() { return std::string(\"%s/%s\"); }\n",
            g_pkg.c_str(), g_name.c_str());
    fprintf(f, "  static std::string get_md5sum() { return std::string(\"%s\"); }\n",
            md5sum.c_str());
    fprintf(f, "  virtual void deserialize_request(uint8_t *read_ptr)\n"
               "  {\n    request.deserialize(read_ptr);\n  }\n");
    fprintf(f, "  virtual void serialize_request(uint8_t *write_ptr)\n"
               "  {\n    request.serialize(write_ptr);\n  }\n");
    fprintf(f, "  virtual void deserialize_response(uint8_t *read_ptr)\n"
               "  {\n    response.deserialize(read_ptr);\n  }\n");
    fprintf(f, "  virtual void serialize_response(uint8_t *write_ptr)\n"
               "  {\n    response.serialize(write_ptr);\n  }\n");
    fprintf(f, "  virtual uint32_t response_serlen()\n"
               "  {\n    return response.serializationLength();\n  }\n");
    fprintf(f, "  virtual uint32_t request_serlen()\n"
               "  {\n    return request.serializationLength();\n  }\n");
    fprintf(f, "};\n\n");
    */
    //fprintf(f, "}\n\n");
    //fprintf(f, "}\n\n");
    //fprintf(f, "#endif\n");

    fclose(f);
    
    // Also write the package.lisp file
    snprintf(fname, PATH_MAX, "%s/_package.lisp", tgt_dir.c_str());
    f = fopen(fname, "w");
    if (!f)
    {
      printf("woah! couldn't write to %s\n", fname);
      exit(7);
    }

    fprintf(f,"(defpackage %s-srv\n"
            "  (:use cl\n"
            "        roslisp-msg-protocol)\n"
            "  (:export\n",
            g_pkg.c_str());

    // Assemble a list of all the .srv files in the same directory, so that
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
      else if (fname.substr(fname.size()-4) != string(".srv"))
        continue;
      else
      {
        string path = fname.substr(0,fname.size()-4);
        string dep_file = g_path + string("/lisp/") + g_pkg + string("/.") + path + string(".asd-dep");
        
        // read dependencies file.
        FILE *dep_fp = fopen(dep_file.c_str(), "r");
        if(dep_fp)
        {
            char dep[255];
            while( !feof(dep_fp) )
            {
                if(fscanf(dep_fp, "%254s", dep) > 0)
                    dependencies.insert( string(dep) );
            }
            fclose(dep_fp);
        }
        
        // string package_file = string("\"") + g_path + string("/lisp/") + 
        //         g_pkg + string("/_package_") + path + string(".lisp\"");
        package_files.push_back(path);
        
        //package_file = string("\"") + g_path + string("/lisp/") + 
                //g_pkg + string("/_package_") + path + string("-response.lisp\"");
        //package_files.push_back(package_file);
        for(unsigned int i=0; i< path.size(); i++)
          path[i] = toupper(path[i]);
        fprintf(f,"   \"%s\"\n", path.c_str());
        fprintf(f,"   \"<%s-REQUEST>\"\n", path.c_str());
        fprintf(f,"   \"<%s-RESPONSE>\"\n", path.c_str());
      }
    }
    fprintf(f,"  ))\n\n");
    
    // for(list<string>::iterator it = package_files.begin();
    //     it != package_files.end();
    //     ++it)
    //   fprintf(f, "(roslisp:load-if-necessary %s)\n", (*it).c_str());

    fclose(f);

        // Emmit asd file.
    snprintf(fname, PATH_MAX, "%s/%s-srv.asd", tgt_dir.c_str(), g_pkg.c_str());
    f = fopen(fname, "w");
    if (!f)
    {
      printf("woah! couldn't write to %s\n", fname);
      exit(7);
    }
    fprintf(f, "\n(in-package :asdf)\n\n");
    fprintf(f, "(defsystem \"%s-srv\"\n"
        "  :default-component-class roslisp-msg-source-file\n"
        "  :depends-on (:roslisp-msg-protocol :roslisp-utils", g_pkg.c_str());

    for(set<string>::iterator it = dependencies.begin();
        it != dependencies.end();
        ++it )
    {
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
    printf("usage: gensrv SRV1 [SRV2] ...\n");
    return 1;
  }
  srv_gen gen;
  for (int i = 1; i < argc; i++)
    gen.process_file(argv[i]);
  return 0;
}

