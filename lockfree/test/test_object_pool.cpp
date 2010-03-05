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

#include <gtest/gtest.h>

#include "lockfree/object_pool.h"

#include <set>

using namespace lockfree;

TEST(ObjectPool, oneElement)
{
  ObjectPool<uint32_t> pool(1, 5);

  boost::shared_ptr<uint32_t> item = pool.allocate();
  ASSERT_TRUE(item);
  EXPECT_EQ(*item, 5UL);
  *item = 6;
  ASSERT_FALSE(pool.allocate());
  item.reset();
  item = pool.allocate();
  ASSERT_TRUE(item);
  EXPECT_EQ(*item, 6UL);
}

TEST(ObjectPool, multipleElements)
{
  const uint32_t count = 5;
  ObjectPool<uint32_t> pool(count, 5);

  std::vector<boost::shared_ptr<uint32_t> > items;
  items.reserve(count);

  for (uint32_t i = 0; i < count; ++i)
  {
    items.push_back(pool.allocate());
    ASSERT_TRUE(items[i]);
    EXPECT_EQ(*items[i], 5UL);
    *items[i] = i;
  }

  ASSERT_FALSE(pool.allocate());
  items.pop_back();
  items.push_back(pool.allocate());
  ASSERT_TRUE(items.back());
  ASSERT_FALSE(pool.allocate());

  std::set<boost::shared_ptr<uint32_t> > set;
  set.insert(items.begin(), items.end());
  EXPECT_EQ(set.size(), count);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
