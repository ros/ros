///////////////////////////////////////////////////////////////////////////////
// The roscpp_tutorials package tries to show off the roscpp c++ client library 
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
#include "ros/time.h"

int main(int argc, char **argv)
{
  // ros::Duration objects are constructed with two parameters: the first
  // is the number of seconds in the duration, and the second is the number
  // of nanoseconds.
  // Once you have this object constructed, you can call its sleep() method
  // to sleep for its duration. (The underlying implementation is nanosleep)
  printf("sleeping for one second\n");
  ros::Duration(1, 0).sleep();

  // This just shows how you have to have to use a lot of zeros to get a half
  // second in nanoseconds.
  printf("sleeping for a half second\n");
  ros::Duration(0, 500000000).sleep();

  // This usage constructs a ros::Duration object and keeps it around, calling
  // the sleep() function repeatedly.
  ros::Duration tenth(0, 100000000); // 0.1 seconds
  for (int i = 0; i < 5; i++)
  {
    printf("sleeping for a tenth of a second\n");
    tenth.sleep();
  }

  
  return 0;
}

