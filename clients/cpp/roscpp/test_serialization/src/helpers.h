/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TEST_ROSCP_SERIALIZATION_HELPERS_H
#define TEST_ROSCP_SERIALIZATION_HELPERS_H

#include <boost/shared_array.hpp>
#include <ros/serialization.h>

namespace roscpp
{
namespace ser = ros::serialization;
namespace mt = ros::message_traits;
typedef boost::shared_array<uint8_t> Array;

template<typename T>
Array serializeAndDeserialize(const T& ser_val, T& deser_val)
{
  uint32_t len = ser::serializationLength(ser_val);
  boost::shared_array<uint8_t> b(new uint8_t[len]);
  ser::OStream ostream(b.get(), len);
  ser::serialize(ostream, ser_val);
  ser::IStream istream(b.get(), len);
  ser::deserialize(istream, deser_val);

  return b;
}

template<class T> class Allocator;

// specialize for void:
template<>
class Allocator<void>
{
public:
  typedef void* pointer;
  typedef const void* const_pointer;
  // reference to void members are impossible.
  typedef void value_type;

  template<class U>
  struct rebind
  {
    typedef Allocator<U> other;
  };
};

template<class T>
class Allocator
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
    typedef Allocator<U> other;
  };

  Allocator() throw ()
  {
  }
  template<class U>
  Allocator(const Allocator<U>& u) throw ()
  {
  }

  ~Allocator() throw ()
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
  pointer allocate(size_type n, Allocator<void>::const_pointer hint = 0)
  {
    return reinterpret_cast<pointer> (malloc(n * sizeof(T)));
  }
  void deallocate(pointer p, size_type n)
  {
    free(p);
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

template<class T1, class T2>
inline bool operator==(const Allocator<T1>& a1, const Allocator<T2>& a2) throw ()
{
  return true;
}

template<class T1, class T2>
inline bool operator!=(const Allocator<T1>& a1, const Allocator<T2>& a2) throw ()
{
  return false;
}

}

#endif // TEST_ROSCP_SERIALIZATION_HELPERS_H
