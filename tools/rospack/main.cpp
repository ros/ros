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

/* Author: Morgan Quigley, Brian Gerkey */


#include <stdexcept>
#include "rospack/rospack.h"

int main(int argc, char **argv)
{
  if (argc <= 1)
  {
    fputs(rospack::ROSPack::usage(), stderr);
    return 0;
  }
  int ret;
  bool quiet;
  try
  {
    // Declare ROSPack instance inside the try block because its
    // constructor can throw (e.g., when ROS_ROOT isn't set).
    rospack::ROSPack rp;
    // Separate try block for running the command, to allow for suppressing
    // error output when -q is given.
    try
    {
      ret = rp.run(argc, argv);
      printf("%s", rp.getOutput().c_str());
    }
    catch(std::runtime_error &e)
    {
      // Return code is -1 no matter what, but don't rethrow if we were
      // asked to be quiet.
      ret = -1;
      if(!rp.is_quiet())
        throw;
    }
  }
  catch(std::runtime_error &e)
  {
    fprintf(stderr, "[rospack] %s\n", e.what());
    ret = -1;
  }

  return ret;
}
