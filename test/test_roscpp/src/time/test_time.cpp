///////////////////////////////////////////////////////////////////////////////
// The test_roscpp package has a few tests of the roscpp c++ client library 
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

#include "ros/time.h"
#include <cstdio>
#include <iostream>

using namespace ros;

int main(int argc, char **argv)
{
  Time t_start = Time::now(), t_prev = Time::now();
  Duration d;
  for (int i = 0; i < 10; i++)
  {
    Time t = Time::now();
    printf("t = %f\n", t.toSec());
    d = t - t_prev;
    printf("d = %f\n", d.toSec());
    t_prev = t;
  }
  Duration d2(123, 456000);
  double d3 = d2.toSec();
  printf("d3 = %f\n", d3);
  Duration d4 = ros::Duration().fromSec(d3);
  printf("d4 = %f\n", d4.toSec());

  Duration d5(0,999999999);
  Duration d6(0,        1);
  printf("d5+d6=%f\n", (d5 + d6).toSec());

  Duration eqd1(1,2);
  Duration eqd2(1,2);
  Duration eqd3(2,3);
  printf("d1=d2: %d\n d1=d3: %d\n", eqd1==eqd2, eqd1==eqd3);

  Time eqt1(1,2), eqt2(1,2), eqt3(2,3);
  printf("eqt1=eqt2: %d\n eqt1=eqt3: %d\n", eqt1==eqt2, eqt1==eqt3);

  Time time1(0, 1);
  Time time2(0, 2);

  std::cout << time1 << " - " << time2 << " = " << time1-time2 << std::endl;
  std::cout << time2 << " - " << time1 << " = " << time2-time1 << std::endl;
  
  return 0;
}

