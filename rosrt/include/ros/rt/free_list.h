
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

#ifndef ROSRT_FREE_LIST_H
#define ROSRT_FREE_LIST_H

#include "aligned_alloc.h"

#include <ros/assert.h>
#include <ros/atomic.h>

#include <boost/array.hpp>

#if FREE_LIST_DEBUG
#include <boost/thread.hpp>
#endif

#define ROSRT_CACHELINE_SIZE 64 // TODO: actually determine this.

#if FREE_LIST_DEBUG
#define FREELIST_DEBUG_YIELD() sched_yield()
#else
#define FREELIST_DEBUG_YIELD()
#endif

namespace ros
{
namespace rt
{

/**
 * \brief A lock-free (*not* wait-free) freelist implemented with CAS
 */
class FreeList
{
public:
  FreeList();
  FreeList(uint32_t block_size, uint32_t block_count);
  ~FreeList();

  void initialize(uint32_t block_size, uint32_t block_count);

  void* allocate();
  void free(void* t);

  template<typename T>
  void constructAll(const T& tmpl)
  {
    for (uint32_t i = 0; i < block_count_; ++i)
    {
      new (blocks_ + (i * block_size_)) T(tmpl);
    }
  }

  template<typename T>
  void constructAll()
  {
    for (uint32_t i = 0; i < block_count_; ++i)
    {
      new (blocks_ + (i * block_size_)) T();
    }
  }

  template<typename T>
  void destructAll()
  {
    for (uint32_t i = 0; i < block_count_; ++i)
    {
      reinterpret_cast<T*>(blocks_ + (i * block_size_))->~T();
    }
  }

#if FREE_LIST_DEBUG
  struct Debug;
  Debug* getDebug()
  {
    return debug_.get();
  }
#endif

private:

  inline uint32_t getTag(uint64_t val)
  {
    return (uint32_t)(val >> 32);
  }

  inline uint32_t getVal(uint64_t val)
  {
    return (uint32_t)val & 0xffffffff;
  }

  inline void setTag(uint64_t& val, uint32_t tag)
  {
    val = getVal(val) | (uint64_t)tag << 32;
  }

  inline void setVal(uint64_t& val, uint32_t v)
  {
    val = ((uint64_t)getTag(val) << 32) | v;
  }

  uint8_t* blocks_;
  atomic_uint32_t* next_;
  atomic_uint64_t head_;

  uint32_t block_size_;
  uint32_t block_count_;

#if FREE_LIST_DEBUG
  struct Debug
  {
    enum
    {
      Alloc,
      Free
    };

    struct Item
    {
      Item()
      : head(0xffffffff)
      , new_head(0xffffffff)
      , addr(0)
      , op(0xff)
      , success(0)
      {}
      ros::WallTime time;
      uint64_t head;
      uint64_t new_head;
      uint8_t* addr;
      uint8_t op;
      uint8_t success;
    };

    std::vector<Item> items;
    std::string thread;
  };

  void initDebug()
  {
    if (!debug_.get())
    {
      debug_.reset(new Debug);
      std::stringstream ss;
      ss << boost::this_thread::get_id();
      debug_->thread = ss.str();
    }
  }

  boost::thread_specific_ptr<Debug> debug_;
#endif
};

} // namespace rt
} // namespace ros

#endif // ROSRT_FREE_LIST_H
