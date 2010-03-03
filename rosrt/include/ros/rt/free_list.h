
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
 * \brief A lock-free (*not* wait-free) statically-sized free-list implemented with CAS
 *
 * FreeList is implemented as a forward-linked list using indices instead of pointers.
 * The array of blocks is allocated separately from the array of next indices, and both
 * are always allocated aligned to a cache line.
 *
 * Indices are stored as 32-bits with a 64-bit head index whose upper 32-bits are tagged
 * to avoid ABA problems
 */
class FreeList
{
public:
  /**
   * \brief Default constructor.  You must call initialize() if you use this constructor.
   */
  FreeList();
  /**
   * \brief Constructor with initialization
   * \param block_size The size of each block allocate() will return
   * \param block_count The number of blocks to allocate
   */
  FreeList(uint32_t block_size, uint32_t block_count);
  ~FreeList();

  /**
   * \brief Initialize this FreeList.  Only use if you used to default constructor
   * \param block_size The size of each block allocate() will return
   * \param block_count The number of blocks to allocate
   */
  void initialize(uint32_t block_size, uint32_t block_count);

  /**
   * \brief Allocate a single block from this FreeList
   * \return 0 if all blocks are allocated, a pointer to a memory block of size block_size_ otherwise
   */
  void* allocate();
  /**
   * \brief Free a block of memory allocated from this FreeList
   * \param mem The block to be freed
   */
  void free(void* mem);
  /**
   * \brief Returns whether or not this FreeList owns a block of memory
   * \param mem the block to check
   * \return true if this FreeList owns the block, false otherwise
   */
  bool owns(void* mem);

  /**
   * \brief Construct all the blocks with a specific template.  If you call this you must call
   * destructAll() prior to destroying this FreeList
   * \param tmpl The object template to use
   *
   * \note sizeof(T) must equal block_size_
   */
  template<typename T>
  void constructAll(const T& tmpl)
  {
    ROS_ASSERT(sizeof(T) == block_size_);
    for (uint32_t i = 0; i < block_count_; ++i)
    {
      new (blocks_ + (i * block_size_)) T(tmpl);
    }
  }

  /**
   * \brief Construct all the blocks with a default constructor.  If you call this you must call
   * destructAll() prior to destroying this FreeList
   * \note sizeof(T) must equal block_size_
   */
  template<typename T>
  void constructAll()
  {
    ROS_ASSERT(sizeof(T) == block_size_);
    for (uint32_t i = 0; i < block_count_; ++i)
    {
      new (blocks_ + (i * block_size_)) T();
    }
  }

  /**
   * \brief Destruct all the objects in this FreeList.  You must have called constructAll() first.
   * \note sizeof(T) must equal block_size_
   */
  template<typename T>
  void destructAll()
  {
    ROS_ASSERT(sizeof(T) == block_size_);
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
