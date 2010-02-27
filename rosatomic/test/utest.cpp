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

#include "ros/atomic.h"
#include <typeinfo>

#include <gtest/gtest.h>

using namespace ros;


template<typename T>
void test_atomic_arithmetic(void)
{
        atomic<T> i(41);

        T n;

        fprintf(stderr, "Type=%s, size=%ld, atomic_size=%ld, lockfree=%d\n",
                typeid(T).name(), (long)sizeof(n), (long)sizeof(i), i.is_lock_free());

        ASSERT_TRUE(sizeof(i)>=sizeof(n));

        bool success;

        n=i++;
        ASSERT_TRUE(i==42);
        ASSERT_TRUE(n==41);

        n=i--;
        ASSERT_TRUE(n==42);
        ASSERT_TRUE(i==41);

        n=++i;
        ASSERT_TRUE(i==42);
        ASSERT_TRUE(n==42);

        n=--i;
        ASSERT_TRUE(n==41);
        ASSERT_TRUE(i==41);

        n=i.fetch_and(15);
        ASSERT_TRUE(n==41);
        ASSERT_TRUE(i==9);

        n=i.fetch_or(17);
        ASSERT_TRUE(n==9);
        ASSERT_TRUE(i==25);

        n=i.fetch_xor(3);
        ASSERT_TRUE(n==25);
        ASSERT_TRUE(i==26);

        n=i.exchange(12);
        ASSERT_TRUE(n==26);
        ASSERT_TRUE(i==12);

        n=12;
        success=i.compare_exchange_strong(n, 17);
        ASSERT_TRUE(success);
        ASSERT_TRUE(n==12);
        ASSERT_TRUE(i==17);

        n=12;
        success=i.compare_exchange_strong(n, 19);
        ASSERT_TRUE(!success);
        ASSERT_TRUE(n==17);
        ASSERT_TRUE(i==17);
}

template<typename T>
void test_atomic_base(void)
{
        atomic<T> i;
        T n;

        fprintf(stderr, "Type=%s, size=%ld, atomic_size=%ld, lockfree=%d\n",
                typeid(T).name(), (long)sizeof(n), (long)sizeof(i), i.is_lock_free());

        ASSERT_TRUE(sizeof(i)>=sizeof(n));

        bool success;

        i.store((T)0);
        n=(T)40;
        success=i.compare_exchange_strong(n, (T)44 /*boost::memory_order_relaxed*/);
        ASSERT_TRUE(!success);
        ASSERT_TRUE(n==(T)0);
        ASSERT_TRUE(i.load()==(T)0);

        n=(T)0;
        success=i.compare_exchange_strong(n, (T)44);
        ASSERT_TRUE(success);
        ASSERT_TRUE(n==(T)0);
        ASSERT_TRUE(i.load()==(T)44);

        n=i.exchange((T)20);
        ASSERT_TRUE(n==(T)44);
        ASSERT_TRUE(i.load()==(T)20);
}

template<typename T>
void test_atomic_ptr(void)
{
        test_atomic_base<T *>();

        T array[10], *p;
        atomic<T *> ptr;

        ptr=&array[0];

        p=ptr++;
        ASSERT_TRUE(p==&array[0]);
        ASSERT_TRUE(ptr==&array[1]);
        p=++ptr;
        ASSERT_TRUE(p==&array[2]);
        ASSERT_TRUE(ptr==&array[2]);

        p=ptr.fetch_add(4);
        ASSERT_TRUE(p==&array[2]);
        ASSERT_TRUE(ptr==&array[6]);

        p=ptr.fetch_sub(4);
        ASSERT_TRUE(p==&array[6]);
        ASSERT_TRUE(ptr==&array[2]);

        p=ptr--;
        ASSERT_TRUE(p==&array[2]);
        ASSERT_TRUE(ptr==&array[1]);
        p=--ptr;
        ASSERT_TRUE(p==&array[0]);
        ASSERT_TRUE(ptr==&array[0]);
}

template<>
void test_atomic_base<bool>(void)
{
        atomic<bool> i;
        bool n;

        fprintf(stderr, "Type=bool, size=%ld, atomic_size=%ld, lockfree=%d\n",
                (long)sizeof(n), (long)sizeof(i), i.is_lock_free());

        ASSERT_TRUE(sizeof(i)>=sizeof(n));

        bool success;
        i=false;
        n=true;
        success=i.compare_exchange_strong(n, true);
        ASSERT_TRUE(!success);
        ASSERT_TRUE(n==false);
        ASSERT_TRUE(i==false);

        n=false;
        success=i.compare_exchange_strong(n, true);
        ASSERT_TRUE(success);
        ASSERT_TRUE(n==false);
        ASSERT_TRUE(i==true);

        n=i.exchange(false);
        ASSERT_TRUE(n==true);
        ASSERT_TRUE(i==false);
}

void test_atomic_flag()
{
        atomic_flag f(0);

        ASSERT_TRUE(!f.test_and_set());
        ASSERT_TRUE(f.test_and_set());
        f.clear();
        ASSERT_TRUE(!f.test_and_set());
}

struct Compound {
        int i;

        inline bool operator==(const Compound &c) const {return i==c.i;}
};

void test_atomic_struct(void)
{
        atomic<Compound> i;
        Compound n;

        Compound zero={0}, one={1}, two={2};

        ASSERT_TRUE(sizeof(i)>=sizeof(n));

        bool success;

        i.store(zero);
        n=one;
        success=i.compare_exchange_strong(n, two);
        ASSERT_TRUE(!success);
        ASSERT_TRUE(n==zero);
        ASSERT_TRUE(i.load()==zero);

        n=zero;
        success=i.compare_exchange_strong(n, two);
        ASSERT_TRUE(success);
        ASSERT_TRUE(n==zero);
        ASSERT_TRUE(i.load()==two);

        n=i.exchange(one);
        ASSERT_TRUE(n==two);
        ASSERT_TRUE(i.load()==one);
}

enum TestEnum {
        Foo, Bar
};

void test_fence()
{
        atomic_thread_fence(memory_order_acquire);
}

TEST(Atomic, all)
{
  test_atomic_arithmetic<char>();
  test_atomic_arithmetic<signed char>();
  test_atomic_arithmetic<unsigned char>();
  test_atomic_arithmetic<uint8_t>();
  test_atomic_arithmetic<int8_t>();
  test_atomic_arithmetic<short>();
  test_atomic_arithmetic<unsigned short>();
  test_atomic_arithmetic<uint16_t>();
  test_atomic_arithmetic<int16_t>();
  test_atomic_arithmetic<int>();
  test_atomic_arithmetic<unsigned int>();
  test_atomic_arithmetic<uint32_t>();
  test_atomic_arithmetic<int32_t>();
  test_atomic_arithmetic<long>();
  test_atomic_arithmetic<unsigned long>();
  test_atomic_arithmetic<uint64_t>();
  test_atomic_arithmetic<int64_t>();
  test_atomic_arithmetic<long long>();
  test_atomic_arithmetic<unsigned long long>();

  test_atomic_struct();

  test_atomic_base<void *>();
  test_atomic_ptr<int>();
  test_atomic_base<bool>();
  test_atomic_base<TestEnum>();

  atomic_thread_fence(memory_order_seq_cst);

  test_fence();

  test_atomic_flag();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
