/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*
* Author: Ethan Dreyfuss
*********************************************************************/

#ifndef PROCESS_INFO_H
#define PROCESS_INFO_H

#include <unistd.h> //for getpid()

#include <libgtop-2.0/glibtop.h>
#include <libgtop-2.0/glibtop/procmem.h>
#include <libgtop-2.0/glibtop/procmap.h>

namespace process_info
{

  /**
   * \brief Computes the current memory usage in MiB (megabytes)
   * This function traverses a map of the memory in use by the process and
   * totals it all up.  It should give the same results as System Monitor's
   * memory display, since it uses the same technique.
   *
   * \param process_id    The process ID for which to get the memory usage, defaults to the current process ID.
   */
  float getMemoryUsage(pid_t process_id = getpid())
  {
    //Get the process map
    glibtop_proc_map buf;
    glibtop_map_entry *maps;
    maps = glibtop_get_proc_map(&buf, process_id);

    //Iterate through the map and sum up the memory used
    int total_mem = 0;
    for(unsigned int i=0; i < buf.number; i++) 
    {
    #ifdef __linux__
      total_mem += maps[i].private_dirty;
    #else
      if(maps[i].perm & GLIBTOP_MAP_PERM_WRITE)
        total_mem += maps[i].size;
    #endif
    }

    //Free the process map
    g_free(maps);

    return ((float)total_mem/(float)(1024*1024));
  }

} //end namespace process_info

#endif //PROCESS_INFO_H
