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

#include <csignal>
#include <cstdlib>
#include "console_master.h"

int main(int argc, char **argv)
{
  signal(SIGPIPE, SIG_IGN);
  int port = 11311;
  if (argc > 1) 
  {
    // todo: incorporate a real options parser. for now, if the argument
    // is all digits, then sanity-check it and treat it as the startup port.
    if (argc > 2)
      printf("syntax: botherder [PORTNUMBER]\nAll other arguments ignored.\n");
    bool port_arg_ok = true;
    for (char *c = argv[1]; *c; c++)
    {
      if (!isdigit(*c))
      {
        printf("The first parameter to botherder must be an integer that \n"
               "will be interpreted as the botherder port number, overriding\n"
               "the default port 11311. Note that doing so will cause\n"
               "problems with the client libraries, as they must be \n"
               "redirected to this new port number\n");
        port_arg_ok = false;
        break;
      }
    }
    if (port_arg_ok)
    {
      int port_arg = atoi(argv[1]);
      if (port_arg < 1024 || port_arg > 65535)
      {
        printf("The port argument must be a user-accessible port number\n"
               "(i.e. between 1025 and 65535)\n");
        return -1;
      }
      else
        port = atoi(argv[1]);
    }
  }
  console_master m(port);
  m.spin();
  return 0;
}

