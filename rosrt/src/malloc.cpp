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

#include <ros/rt/malloc_wrappers.h>

#include <ros/assert.h>
#include <ros/atomic.h>

#include <boost/thread.hpp>

#include <iostream>
#include <dlfcn.h>

#if defined(WIN32)
#define STATIC_TLS_KW __declspec(thread)
#define HAS_TLS_KW 1
#elif defined(__APPLE__)
#define HAS_TLS_KW 0
#else
#define STATIC_TLS_KW __thread
#define HAS_TLS_KW 1
#endif

#if !HAS_TLS_KW
#include <pthread.h>
#define MAX_ALLOC_INFO 1000
#endif

namespace ros
{
namespace rt
{
namespace detail
{

#if HAS_TLS_KW
STATIC_TLS_KW uint64_t g_mallocs = 0;
STATIC_TLS_KW uint64_t g_reallocs = 0;
STATIC_TLS_KW uint64_t g_callocs = 0;
STATIC_TLS_KW uint64_t g_memaligns = 0;
STATIC_TLS_KW uint64_t g_frees = 0;
STATIC_TLS_KW uint64_t g_total_ops = 0;
STATIC_TLS_KW uint64_t g_total_memory_allocated = 0;
STATIC_TLS_KW bool g_break_on_alloc_or_free = false;
#else // HAS_TLS_KW

pthread_key_t g_tls_key;
bool g_tls_key_initialized = false;

AllocInfo g_thread_alloc_info[MAX_ALLOC_INFO];
atomic_bool g_alloc_info_used[MAX_ALLOC_INFO];

void tlsDestructor(void* mem)
{
  AllocInfo* info = reinterpret_cast<AllocInfo*>(mem);
  uint32_t index = info - g_thread_alloc_info;
  ROS_ASSERT(index < MAX_ALLOC_INFO);
  g_alloc_info_used[index].store(false);
}

struct MallocTLSInit
{
  MallocTLSInit()
  {
    int ret = pthread_key_create(&g_tls_key, tlsDestructor);
    ROS_ASSERT_MSG(!ret, "Failed to create TLS");

    for (size_t i = 0; i < MAX_ALLOC_INFO; ++i)
    {
      g_alloc_info_used[i].store(false);
    }

    g_tls_key_initialized = true;
  }

  ~MallocTLSInit()
  {
    g_tls_key_initialized = false;

    pthread_key_delete(g_tls_key);
  }
};
MallocTLSInit g_malloc_tls_init;

AllocInfo* allocateAllocInfo()
{
  if (!g_tls_key_initialized)
  {
    return 0;
  }

  void* info = pthread_getspecific(g_tls_key);
  if (!info)
  {
    for (size_t i = 0; i < MAX_ALLOC_INFO; ++i)
    {
      if (g_alloc_info_used[i].exchange(true) == false)
      {
        info = g_thread_alloc_info + i;
        pthread_setspecific(g_tls_key, info);
        break;
      }
    }
  }

  return reinterpret_cast<AllocInfo*>(info);
}

#endif // !HAS_TLS_KW

} // namespace malloc_tls

AllocInfo getThreadAllocInfo()
{
  AllocInfo info;

#if HAS_TLS_KW
  info.mallocs = detail::g_mallocs;
  info.callocs = detail::g_callocs;
  info.reallocs = detail::g_reallocs;
  info.memaligns = detail::g_memaligns;
  info.frees = detail::g_frees;
  info.total_ops = detail::g_total_ops;
  info.total_memory_allocated = detail::g_total_memory_allocated;
  info.break_on_alloc_or_free = detail::g_break_on_alloc_or_free;
#else
  AllocInfo* tls = detail::allocateAllocInfo();
  if (tls)
  {
    info = *tls;
  }
#endif
  return info;
}

void resetThreadAllocInfo()
{
#if HAS_TLS_KW
  detail::g_mallocs = 0;
  detail::g_reallocs = 0;
  detail::g_callocs = 0;
  detail::g_memaligns = 0;
  detail::g_frees = 0;
  detail::g_total_ops = 0;
  detail::g_total_memory_allocated = 0;
#else
  AllocInfo* info = detail::allocateAllocInfo();
  if (info)
  {
    *info = AllocInfo();
  }
#endif
}

void setThreadBreakOnAllocOrFree(bool b)
{
#if HAS_TLS_KW
  detail::g_break_on_alloc_or_free = b;
#else
  AllocInfo* info = detail::allocateAllocInfo();
  if (info)
  {
    info->break_on_alloc_or_free = b;
  }
#endif
}

} // namespace rt
} // namespace ros

extern "C"
{

typedef void* (*MallocType)(size_t size);
typedef void* (*CallocType)(size_t nmemb, size_t size);
typedef void* (*ReallocType)(void *ptr, size_t size);
typedef void* (*MemalignType)(size_t boundary, size_t size);
typedef int (*PosixMemalignType)(void **memptr, size_t alignment, size_t size);
typedef void (*FreeType)(void* ptr);

#if HAS_TLS_KW
#define UPDATE_ALLOC_INFO(result, size, type) \
  if (result) \
  { \
    ros::rt::detail::g_total_memory_allocated += size; \
  } \
\
  ++ros::rt::detail::g_##type; \
  ++ros::rt::detail::g_total_ops; \
\
  if (ros::rt::detail::g_break_on_alloc_or_free) \
  { \
    std::cerr << "Issuing break due to break_on_alloc_or_free being set" << std::endl; \
    ROS_ISSUE_BREAK(); \
  }
#else
#define UPDATE_ALLOC_INFO(result, size, type) \
  ros::rt::AllocInfo* tls = ros::rt::detail::allocateAllocInfo(); \
  if (tls) \
  { \
    if (result) \
    { \
      tls->total_memory_allocated += size; \
    } \
  \
    ++tls->type; \
    ++tls->total_ops; \
  \
    if (tls->break_on_alloc_or_free) \
    { \
      std::cerr << "Issuing break due to break_on_alloc_or_free being set" << std::endl; \
      ROS_ISSUE_BREAK(); \
    } \
  }
#endif

void* malloc(size_t size)
{
  static MallocType original_function = reinterpret_cast<MallocType>(dlsym(RTLD_NEXT, "malloc"));

  void* result = original_function(size);

  UPDATE_ALLOC_INFO(result, size, mallocs);

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

  uint32_t size = 0;
  void* result = 0;
  UPDATE_ALLOC_INFO(result, size, frees);
}

void __libc_free(void* ptr)
{
  return free(ptr);
}

// dlsym uses calloc so it has to be treated specially. Solution found at http://crbug.com/28244
static void* nullCalloc(size_t nmemb, size_t size)
{
  return 0;
}

void* calloc(size_t nmemb, size_t size)
{
  static CallocType original_function = 0;
  if (original_function == 0)
  {
      original_function = nullCalloc;
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


