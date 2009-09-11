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
#include <stdexcept>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/param.h>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <sstream>
#include "utils.h"
#include "msgspec.h"
using namespace std;

class srv_gen;
srv_gen *g_srv_gen = NULL;

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
    string tgt_dir = g_path;
    vector<string> dirs;
    dirs.push_back("java");
//    dirs.push_back("com");
//    dirs.push_back("willowgarage");
    dirs.push_back("ros");
    dirs.push_back("pkg");
    dirs.push_back(g_pkg);
    dirs.push_back("srv");
//    string tgt_dir = g_path + string("/java/") + g_pkg;

    for(vector<string>::iterator di = dirs.begin(); di != dirs.end(); di++) {
    	tgt_dir += string("/") + *di;
    if (access(tgt_dir.c_str(), F_OK) != 0)
      if (mkdir(tgt_dir.c_str(), 0755) && (errno != EEXIST))
      {
        printf("woah! error from mkdir: [%s]\n", strerror(errno));
        exit(5);
      }
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
    snprintf(fname, PATH_MAX, "%s/%s.java", tgt_dir.c_str(), g_name.c_str());
    FILE *f = fopen(fname, "w");
    if (!f)
    {
      printf("woah! couldn't write to %s\n", fname);
      exit(7);
    }
    string pkg_upcase = to_upper(g_pkg), srv_upcase = to_upper(g_name);
    fprintf(f, "/* auto-generated by gensrv_java from %s.  Do not edit! */\n",
            spec_file);
//    fprintf(f, "\n#include <string>\n"
//	       "#include <cstring>\n"
//	       "#include <vector>\n"
//         "#include <map>\n\n");
    fprintf(f, "package ros.pkg.%s.srv;\n\n", g_pkg.c_str());

    fprintf(f, "import java.nio.ByteBuffer;\n\n");

//    request_spec.emit_cpp_includes(f);
//    response_spec.emit_cpp_includes(f);

    fprintf(f, "public class %s extends ros.communication.Service<%s.Request, %s.Response> \n{\n\n", g_name.c_str(), g_name.c_str(), g_name.c_str());

    fprintf(f, "public static String __s_getDataType() { return \"%s/%s\"; }\n", g_pkg.c_str(), g_name.c_str());
    fprintf(f, "public static String __s_getMD5Sum() { return \"%s\"; }\n\n", md5sum.c_str());
    fprintf(f, "public String getDataType() { return %s.__s_getDataType(); }\n", g_name.c_str());
    fprintf(f, "public String getMD5Sum() { return %s.__s_getMD5Sum(); }\n\n", g_name.c_str());
    string srv_name = g_name;
    g_name = "Request";
    request_spec.emit_cpp_class(f, true, srv_name);
    g_name = "Response";
    response_spec.emit_cpp_class(f, true, srv_name);
    g_name = srv_name;

    fprintf(f, "public %s.Request createRequest() { return new %s.Request(); }", g_name.c_str(), g_name.c_str());
    fprintf(f, "public %s.Response createResponse() { return new %s.Response(); }", g_name.c_str(), g_name.c_str());
//    fprintf(f, "Request request;\n");
//    fprintf(f, "Response response;\n\n");
    fprintf(f, "}\n\n");

    fclose(f);
  }
};

int main(int argc, char **argv)
{
  if (argc <= 1)
  {
    printf("usage: gensrv_java SRV1 [SRV2] ...\n");
    return 1;
  }
  srv_gen gen;
  for (int i = 1; i < argc; i++)
    gen.process_file(argv[i]);
  return 0;
}

