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

#include <malloc.h>
#include <ros/rt/malloc_wrappers.h>

#include <ros/assert.h>
#include <ros/atomic.h>

#include <boost/thread.hpp>

#include <dlfcn.h>

namespace ros
{
namespace rt
{
namespace malloc_tls
{
boost::thread_specific_ptr<AllocInfo> g_alloc_info;

static AllocInfo* getTLS()
{
  return g_alloc_info.get();
}

void initTLS()
{
  if (!g_alloc_info.get())
  {
    g_alloc_info.reset(new AllocInfo);
  }
}

} // namespace malloc_tls

const AllocInfo* getThreadAllocInfo()
{
  AllocInfo* info = malloc_tls::getTLS();
  ROS_ASSERT_MSG(info, "ros::rt::initThread() or ros::rt::initThreadAllocInfo() have not been called in this thread");
  return info;
}

void resetThreadAllocInfo()
{
  AllocInfo* info = malloc_tls::getTLS();
  ROS_ASSERT_MSG(info, "ros::rt::initThread() or ros::rt::initThreadAllocInfo() have not been called in this thread");
  *info = AllocInfo();
}

void setThreadBreakOnAllocOrFree(bool b)
{
  AllocInfo* info = malloc_tls::getTLS();
  ROS_ASSERT_MSG(info, "ros::rt::initThread() or ros::rt::initThreadAllocInfo() have not been called in this thread");
  info->break_on_alloc_or_free = b;
}

void initThreadAllocInfo()
{
  malloc_tls::initTLS();
}

} // namespace rt
} // namespace ros

extern "C"
{

static bool g_malloc_initialized = false;

typedef void* (*MallocType)(size_t size);
typedef void* (*CallocType)(size_t nmemb, size_t size);
typedef void* (*ReallocType)(void *ptr, size_t size);
typedef void* (*MemalignType)(size_t boundary, size_t size);
typedef int (*PosixMemalignType)(void **memptr, size_t alignment, size_t size);
typedef void (*FreeType)(void* ptr);

#define UPDATE_ALLOC_INFO(result, size, type) \
  if (g_malloc_initialized) \
  { \
    ros::rt::AllocInfo* info = ros::rt::malloc_tls::getTLS(); \
    if (info) \
    { \
      if (result) \
      { \
        info->total_memory_allocated += size; \
      } \
 \
      ++info->type; \
      ++info->total_ops; \
 \
      if (info->break_on_alloc_or_free) \
      { \
        std::cerr << "Issuing break due to break_on_alloc_or_free being set" << std::endl; \
        ROS_ISSUE_BREAK(); \
      } \
    } \
  }

void* malloc(size_t size)
{
  static MallocType original_function = 0;
  static ros::atomic<uint32_t> tls_recursion_count(0);

  if (!original_function)
  {
    original_function = reinterpret_cast<MallocType>(dlsym(RTLD_NEXT, "malloc"));
    g_malloc_initialized = true;
  }

  void* result = original_function(size);

  if (g_malloc_initialized)
  {
    uint32_t rec = tls_recursion_count.fetch_add(1);

    if (rec < 2)
    {
      UPDATE_ALLOC_INFO(result, size, mallocs);
    }

    tls_recursion_count.fetch_sub(1);
  }

  return result;
}

void* __libc_malloc(size_t size)
{
  return malloc(size);
}

void* realloc(void* ptr, size_t size)
{
  static ReallocType original_function = reinterpret_cast<ReallocType>(dlsym(RTLD_NEXT, "realloc"));

  void* result = original_function(ptr, size);

  UPDATE_ALLOC_INFO(result, size, reallocs);

  return result;
}

void* __libc_realloc(void* ptr, size_t size)
{
  return realloc(ptr, size);
}

void* memalign(size_t boundary, size_t size)
{
  static MemalignType original_function = reinterpret_cast<MemalignType>(dlsym(RTLD_NEXT, "memalign"));

  void* result = original_function(boundary, size);

  UPDATE_ALLOC_INFO(result, size, memaligns);

  return result;
}

void* __libc_memalign(size_t boundary, size_t size)
{
  return memalign(boundary, size);
}

void free(void *ptr)
{
  static FreeType original_function = reinterpret_cast<FreeType>(dlsym(RTLD_NEXT, "free"));

  original_function(ptr);

  ros::rt::AllocInfo* info = ros::rt::malloc_tls::getTLS();
  if (info)
  {
    ++info->frees;
    ++info->total_ops;

    if (info->break_on_alloc_or_free)
    {
      ROS_BREAK();
    }
  }
}

void __libc_free(void* ptr)
{
  return free(ptr);
}

// dlsym uses calloc so it has to be treated specially. Solution found at http://crbug.com/28244
static void* null_calloc(size_t nmemb, size_t size)
{
  return 0;
}

void* calloc(size_t nmemb, size_t size)
{
  static CallocType original_function = 0;
  if (original_function == 0)
  {
      original_function = null_calloc;
      original_function = reinterpret_cast<CallocType>(dlsym(RTLD_NEXT, "calloc"));
  }

  void* result = original_function(nmemb, size);

  UPDATE_ALLOC_INFO(result, size * nmemb, callocs);

  return result;
}

void* __libc_calloc(size_t nmemb, size_t size)
{
  return calloc(nmemb, size);
}

int posix_memalign(void** ptr, size_t alignment, size_t size) {
  static PosixMemalignType original_function = reinterpret_cast<PosixMemalignType>(dlsym(RTLD_NEXT, "posix_memalign"));

  int result = original_function(ptr, alignment, size);

  UPDATE_ALLOC_INFO(!result, size, memaligns);

  return result;
}


} // extern "C"


