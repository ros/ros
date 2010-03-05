/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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
*********************************************************************/

#ifndef ROSRT_MALLOC_WRAPPERS_H
#define ROSRT_MALLOC_WRAPPERS_H

#include <ros/types.h>

namespace ros
{
namespace rt
{

struct AllocInfo
{
  AllocInfo()
  : mallocs(0)
  , reallocs(0)
  , callocs(0)
  , memaligns(0)
  , frees(0)
  , total_memory_allocated(0)
  , total_ops(0)
  , break_on_alloc_or_free(false)
  {}

  uint64_t mallocs;
  uint64_t reallocs;
  uint64_t callocs;
  uint64_t memaligns;
  uint64_t frees;

  uint64_t total_memory_allocated;
  uint64_t total_ops;

  bool break_on_alloc_or_free;
};

AllocInfo getThreadAllocInfo();
void resetThreadAllocInfo();
void setThreadBreakOnAllocOrFree(bool b);

}
}

#endif // ROSRT_MALLOC_WRAPPERS_H
