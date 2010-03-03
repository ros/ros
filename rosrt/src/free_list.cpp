
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

#include <ros/rt/free_list.h>

namespace ros
{
namespace rt
{

FreeList::FreeList()
: blocks_(0)
, next_(0)
, block_size_(0)
, block_count_(0)
{
}

FreeList::FreeList(uint32_t block_size, uint32_t block_count)
: blocks_(0)
, next_(0)
, block_size_(0)
, block_count_(0)
{
  initialize(block_size, block_count);
}

FreeList::~FreeList()
{
  for (uint32_t i = 0; i < block_count_; ++i)
  {
    next_[i].~atomic_uint32_t();
  }

  alignedFree(blocks_);
  alignedFree(next_);
}

void FreeList::initialize(uint32_t block_size, uint32_t block_count)
{
  ROS_ASSERT(!blocks_);
  ROS_ASSERT(!next_);

  block_size_ = block_size;
  block_count_ = block_count;

  head_.store(0);

  blocks_ = (uint8_t*)alignedMalloc(block_size * block_count, ROSRT_CACHELINE_SIZE);
  next_ = (atomic_uint32_t*)alignedMalloc(sizeof(atomic_uint32_t) * block_count, ROSRT_CACHELINE_SIZE);

  memset(blocks_, 0xCD, block_size * block_count);

  for (uint32_t i = 0; i < block_count_; ++i)
  {
    new (next_ + i) atomic_uint32_t();

    if (i == block_count_ - 1)
    {
      next_[i].store(0xffffffffUL);
    }
    else
    {
      next_[i].store(i + 1);
    }
  }
}

void* FreeList::allocate()
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
      i.addr = blocks_ + (block_size_ * getVal(head));
      i.success = 1;
      debug_->items.push_back(i);
#endif
      return static_cast<void*>(blocks_ + (block_size_ * getVal(head)));
    }

#if FREE_LIST_DEBUG
      i.success = 0;
      debug_->items.push_back(i);
#endif
  }
}

void FreeList::free(void* t)
{
#if FREE_LIST_DEBUG
  initDebug();
#endif

  uint32_t index = (static_cast<uint8_t*>(t) - blocks_) / block_size_;

  ROS_ASSERT(((static_cast<uint8_t*>(t) - blocks_) % block_size_) == 0);
  ROS_ASSERT(index < block_count_);

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
      i.addr = blocks_ + (block_size_ * index);
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

} // namespace rt
} // namespace ros

