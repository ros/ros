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

#ifndef ROSRT_OBJECT_POOL_H
#define ROSRT_OBJECT_POOL_H

#include "free_list.h"

#include <ros/assert.h>

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/array.hpp>
#include <boost/aligned_storage.hpp>
#include <boost/bind.hpp>

namespace ros
{
namespace rt
{

template<class T> class FixedBlockAllocator;

// specialize for void:
template<>
class FixedBlockAllocator<void>
{
public:
  typedef void* pointer;
  typedef const void* const_pointer;
  // reference to void members are impossible.
  typedef void value_type;

  template<class U>
  struct rebind
  {
    typedef FixedBlockAllocator<U> other;
  };

  FixedBlockAllocator(uint8_t* block, uint32_t size) throw ()
  : block_(block)
  , used_(0)
  , size_(size)
  {
  }

  template<class U>
  FixedBlockAllocator(const FixedBlockAllocator<U>& u) throw ()
  {
    block_ = u.get_block();
    size_ = u.get_size();
    used_ = u.get_used();
  }

  uint32_t get_size() const { return size_; }
  uint8_t* get_block() const { return block_; }
  uint32_t get_used() const { return used_; }

private:
  uint8_t* block_;
  uint32_t used_;
  uint32_t size_;
};

template<class T>
class FixedBlockAllocator
{
public:
  typedef size_t size_type;
  typedef ptrdiff_t difference_type;
  typedef T* pointer;
  typedef const T* const_pointer;
  typedef T& reference;
  typedef const T& const_reference;
  typedef T value_type;

  template<class U>
  struct rebind
  {
    typedef FixedBlockAllocator<U> other;
  };

  FixedBlockAllocator(uint8_t* block, uint32_t size) throw ()
  : block_(block)
  , used_(0)
  , size_(size)
  {
  }

  template<class U>
  FixedBlockAllocator(const FixedBlockAllocator<U>& u) throw ()
  {
    block_ = u.get_block();
    size_ = u.get_size();
    used_ = u.get_used();
  }

  ~FixedBlockAllocator() throw ()
  {
  }

  pointer address(reference r) const
  {
    return &r;
  }
  const_pointer address(const_reference r) const
  {
    return &r;
  }
  size_type max_size() const throw ()
  {
    return ~size_type(0);
  }
  pointer allocate(size_type n, FixedBlockAllocator<void>::const_pointer hint = 0)
  {
    uint32_t to_alloc = n * sizeof(T);
    ROS_ASSERT_MSG(to_alloc <= (size_ - used_), "to_alloc=%d, size_=%d, used_=%d", to_alloc, size_, used_);

    pointer p = reinterpret_cast<pointer>(block_ + used_);
    used_ += to_alloc;
    return p;
  }
  void deallocate(pointer p, size_type n)
  {
  }

  void construct(pointer p, const_reference val)
  {
    new (p) T(val);
  }
  void destroy(pointer p)
  {
    p->~T();
  }

  uint32_t get_size() const { return size_; }
  uint8_t* get_block() const { return block_; }
  uint32_t get_used() const { return used_; }

private:
  uint8_t* block_;
  uint32_t used_;
  uint32_t size_;
};

template<typename T>
class ObjectPool
{
  struct SPStorage
  {
    uint8_t data[64];
  };

  struct Deleter
  {
    Deleter(ObjectPool* pool, SPStorage* storage)
    : pool_(pool)
    , sp_(storage)
    {

    }

    void operator()(T* t)
    {
      pool_->deallocate(t, sp_);
    }

  private:
    ObjectPool* pool_;
    SPStorage* sp_;
  };

public:
  ObjectPool()
  : initialized_(false)
  {
  }

  ObjectPool(uint32_t size, const T& tmpl)
  : initialized_(false)
  {
    initialize(size, tmpl);
  }

  ~ObjectPool()
  {
    freelist_.template destructAll<T>();
    sp_storage_freelist_.template destructAll<SPStorage>();
  }

  void initialize(uint32_t size, const T& tmpl)
  {
    ROS_ASSERT(!initialized_);
    freelist_.initialize(size);
    freelist_.template constructAll<T>(tmpl);
    sp_storage_freelist_.initialize(size);
    sp_storage_freelist_.template constructAll<SPStorage>();
    initialized_ = true;
  }

  boost::shared_ptr<T> allocate()
  {
    ROS_ASSERT(initialized_);

    T* item = static_cast<T*>(freelist_.allocate());
    if (!item)
    {
      return boost::shared_ptr<T>();
    }

    SPStorage* sp_storage_s = static_cast<SPStorage*>(sp_storage_freelist_.allocate());
    ROS_ASSERT(sp_storage_s);

    uint8_t* sp_storage = sp_storage_s->data;
    boost::shared_ptr<T> ptr(item, Deleter(this, sp_storage_s),
                             FixedBlockAllocator<void>(sp_storage, sizeof(SPStorage)));
    return ptr;
  }

private:
  void deallocate(T* t, SPStorage* sp)
  {
    freelist_.free(t);
    sp_storage_freelist_.free(sp);
  }

  bool initialized_;

  FreeList<sizeof(T)> freelist_;
  FreeList<sizeof(SPStorage)> sp_storage_freelist_;
};

}
}

#endif // ROSRT_OBJECT_POOL_H
