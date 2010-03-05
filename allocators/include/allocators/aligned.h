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

#ifndef ALLOCATORS_ALIGNED_H
#define ALLOCATORS_ALIGNED_H

#include <cstdlib>
#include <boost/cstdint.hpp>

namespace allocators
{

// alignedMalloc/alignedFree implementations found at
// http://www.gamasutra.com/view/feature/3942/data_alignment_part_1.php?page=2

/**
 * \brief Allocate memory aligned at on a value.  Memory allocated through alignedMalloc() \b must be
 * freed through alignedFree()
 * \param size The amount of memory to allocate
 * \param alignment The value to align on
 */
inline void* alignedMalloc(size_t size, size_t alignment)
{
  const int pointerSize = sizeof(void*);
  const int requestedSize = size + alignment - 1 + pointerSize;
  void* raw = std::malloc(requestedSize);

  if (!raw)
  {
    return 0;
  }

  void* start = (uint8_t*)raw + pointerSize;
  void* aligned = (void*)(((uintptr_t)((uint8_t*)start+alignment-1)) & ~(alignment-1));
  *(void**)((uint8_t*)aligned-pointerSize) = raw;
  return aligned;
}

/**
 * \brief Free memory allocated through alignedMalloc()
 * \param aligned The memory to free
 */
inline void alignedFree(void* aligned)
{
  if (!aligned)
  {
    return;
  }

  void* raw = *(void**)((uint8_t*)aligned - sizeof(void*));
  std::free(raw);
}

template<class T, size_t align> class AlignedAllocator;

// specialize for void:
template<size_t align>
class AlignedAllocator<void, align>
{
public:
  typedef void* pointer;
  typedef const void* const_pointer;
  // reference to void members are impossible.
  typedef void value_type;

  template<class U>
  struct rebind
  {
    typedef AlignedAllocator<U, align> other;
  };
};

/**
 * \brief An stl-compatible aligned allocator
 * \param T The type of the container element
 * \param align The alignment to allocate on
 */
template<class T, size_t align>
class AlignedAllocator
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
    typedef AlignedAllocator<U, align> other;
  };

  AlignedAllocator() throw ()
  {
  }

  ~AlignedAllocator() throw ()
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
  pointer allocate(size_type n, typename AlignedAllocator<void, align>::const_pointer hint = 0)
  {
    void* mem = alignedMalloc(n * sizeof(T), align);
    if (!mem)
    {
      throw std::bad_alloc();
    }

    return static_cast<pointer>(mem);
  }
  void deallocate(pointer p, size_type n)
  {
    alignedFree(p);
  }

  void construct(pointer p, const_reference val)
  {
    new (p) T(val);
  }
  void destroy(pointer p)
  {
    p->~T();
  }
};

} // namespace allocators

#endif // ALLOCATORS_ALIGNED_H

