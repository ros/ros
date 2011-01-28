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

/* Author: Josh Faust */

/*
 * Test Header serialization/deserialization
 */

#include <gtest/gtest.h>
#include "ros/names.h"

using namespace ros;

TEST(Names, validation)
{
  std::string error;
  EXPECT_TRUE(names::validate("", error));
  EXPECT_TRUE(names::validate("hello", error));
  EXPECT_TRUE(names::validate("/hello/asdf", error));
  EXPECT_TRUE(names::validate("hello/fdas", error));
  EXPECT_TRUE(names::validate("/hello", error));
  EXPECT_TRUE(names::validate("~hello", error));
  EXPECT_TRUE(names::validate("/hello_world/blah_world", error));
  EXPECT_TRUE(names::validate("hello123_421", error));
  EXPECT_FALSE(names::validate("123hello", error));
  EXPECT_FALSE(names::validate("_hello", error));
  EXPECT_FALSE(names::validate("h~ello", error));
  EXPECT_FALSE(names::validate("%hello", error));
  EXPECT_FALSE(names::validate("h ello", error));
  EXPECT_FALSE(names::validate("h*ello", error));
  EXPECT_FALSE(names::validate("h?ello", error));
}

TEST(Names, parentNamespace)
{
  //edge casee
  EXPECT_STREQ(std::string("").c_str(), names::parentNamespace("").c_str());
  EXPECT_STREQ(std::string("/").c_str(), names::parentNamespace("/").c_str());
  EXPECT_STREQ(std::string("/").c_str(), names::parentNamespace("/a").c_str());
  EXPECT_STREQ(std::string("/").c_str(), names::parentNamespace("/a/").c_str()); //trailing slash

  //2 long
  EXPECT_STREQ(std::string("/a").c_str(), names::parentNamespace("/a/b").c_str());
  EXPECT_STREQ(std::string("/a").c_str(), names::parentNamespace("/a/b/").c_str()); //trailing slash
  EXPECT_STREQ(std::string("/asdf").c_str(), names::parentNamespace("/asdf/b").c_str());

  //3 long
  EXPECT_STREQ(std::string("/z/a").c_str(), names::parentNamespace("/z/a/b").c_str());
  EXPECT_STREQ(std::string("/z/a").c_str(), names::parentNamespace("/z/a/b/").c_str()); //trailing slash
  EXPECT_STREQ(std::string("/z/asdf").c_str(), names::parentNamespace("/z/asdf/b").c_str());
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


