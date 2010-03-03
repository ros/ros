
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
#include <boost/thread.hpp>

#define ROSRT_CACHELINE_SIZE 64 // TODO: actually determine this.

#define FREELIST_DEBUG_YIELD() sched_yield()

namespace ros
{
namespace rt
{

/**
 * \brief A lock-free (*not* wait-free) freelist implemented with CAS
 */
template<size_t block_size>
class FreeList
{
  typedef std::vector<atomic_uint64_t, AlignedAllocator<atomic_uint64_t, ROSRT_CACHELINE_SIZE> > V_Next;

public:
  FreeList()
  : blocks_(0)
  , next_(0)
  , size_(0)
  {
  }

  FreeList(uint32_t size)
  : blocks_(0)
  , next_(0)
  , size_(0)
  {
    initialize(size);
  }

  ~FreeList()
  {
    for (uint32_t i = 0; i < size_; ++i)
    {
      next_[i].~atomic_uint32_t();
    }

    alignedFree(blocks_);
    alignedFree(next_);
  }

  void initialize(uint32_t size)
  {
    ROS_ASSERT(!blocks_);
    ROS_ASSERT(!next_);

    size_ = size;
    head_.store(0);

    blocks_ = (uint8_t*)alignedMalloc(block_size * size, ROSRT_CACHELINE_SIZE);
    next_ = (atomic_uint32_t*)alignedMalloc(sizeof(atomic_uint32_t) * size, ROSRT_CACHELINE_SIZE);

    memset(blocks_, 0xCD, block_size * size);

    for (uint32_t i = 0; i < size; ++i)
    {
      new (next_ + i) atomic_uint32_t();

      if (i == size - 1)
      {
        next_[i].store(0xffffffffUL);
      }
      else
      {
        next_[i].store(i + 1);
      }
    }
  }

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
#endif

  void* allocate()
  {
#if FREE_LIST_DEBUG
    initDebug();
#endif

    ROS_ASSERT(blocks_);

    while (true)
    {
      uint64_t head = head_.load(memory_order_consume);

#if FREE_LIST_DEBUG
      typename Debug::Item i;
      i.head = head;
      i.time = ros::WallTime::now();
      i.op = Debug::Alloc;
#endif

      if (getVal(head) == 0xffffffffULL)
      {
#if FREE_LIST_DEBUG
        debug_->items.push_back(i);
#endif
        return 0;  // Allocation failed
      }

      FREELIST_DEBUG_YIELD();

      // Load the next index
      uint64_t new_head = next_[getVal(head)].load();

      FREELIST_DEBUG_YIELD();

      // Increment the tag to avoid ABA
      setTag(new_head, getTag(head) + 1);

#if FREE_LIST_DEBUG
      i.new_head = new_head;
#endif

      FREELIST_DEBUG_YIELD();

      // If setting head to next is successful, return the item at next
      if (head_.compare_exchange_strong(head, new_head))
      {
#if FREE_LIST_DEBUG
        i.addr = blocks_ + (block_size * getVal(head));
        i.success = 1;
        debug_->items.push_back(i);
#endif
        return static_cast<void*>(blocks_ + (block_size * getVal(head)));
      }

#if FREE_LIST_DEBUG
        i.success = 0;
        debug_->items.push_back(i);
#endif
    }
  }

  void free(void* t)
  {
#if FREE_LIST_DEBUG
    initDebug();
#endif

    uint32_t index = (static_cast<uint8_t*>(t) - blocks_) / block_size;

    ROS_ASSERT(((static_cast<uint8_t*>(t) - blocks_) % block_size) == 0);
    ROS_ASSERT(index < size_);

    while (true)
    {
      // Load head
      uint64_t head = head_.load(memory_order_consume);

#if FREE_LIST_DEBUG
      typename Debug::Item i;
      i.head = head;
      i.time = ros::WallTime::now();
      i.op = Debug::Free;
#endif

      FREELIST_DEBUG_YIELD();

      uint64_t new_head = head;
      // set new head to the index of the block we're currently freeing
      setVal(new_head, index);
      // Increment the tag to avoid ABA
      setTag(new_head, getTag(new_head) + 1);


      FREELIST_DEBUG_YIELD();

      // Store head as next index for this item
      next_[index].store(getVal(head));

      FREELIST_DEBUG_YIELD();

#if FREE_LIST_DEBUG
      i.new_head = new_head;
#endif

      FREELIST_DEBUG_YIELD();

      // If setting the head to next is successful, return
      if (head_.compare_exchange_strong(head, new_head))
      {
#if FREE_LIST_DEBUG
        i.success = 1;
        i.addr = blocks_ + (block_size * index);
        debug_->items.push_back(i);
#endif
        return;
      }

#if FREE_LIST_DEBUG
        i.success = 0;
        debug_->items.push_back(i);
#endif
    }
  }

  template<typename T>
  void constructAll(const T& tmpl)
  {
    for (uint32_t i = 0; i < size_; ++i)
    {
      new (blocks_ + (i * block_size)) T(tmpl);
    }
  }

  template<typename T>
  void constructAll()
  {
    for (uint32_t i = 0; i < size_; ++i)
    {
      new (blocks_ + (i * block_size)) T();
    }
  }

  template<typename T>
  void destructAll()
  {
    for (uint32_t i = 0; i < size_; ++i)
    {
      reinterpret_cast<T*>(blocks_ + (i * block_size))->~T();
    }
  }

#if FREE_LIST_DEBUG
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

  Debug* getDebug()
  {
    return debug_.get();
  }
#endif

private:

  uint8_t* blocks_;
  atomic_uint32_t* next_;
  atomic_uint64_t head_;

  uint32_t size_;

#if FREE_LIST_DEBUG
  boost::thread_specific_ptr<Debug> debug_;
#endif
};

}
}

#endif // ROSRT_FREE_LIST_H
