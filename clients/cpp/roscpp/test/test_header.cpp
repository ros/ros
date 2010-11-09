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
#include "ros/header.h"

static char g_header_data1[] = "A=B";
static char g_header_data2[] = "AAAAAAAAAAAAA=BBBBBBB\nBBBBBBB";

using namespace ros;

TEST(Header, parse)
{
  Header header;

  uint32_t total_len = 4 + sizeof(g_header_data1) + 4 + sizeof(g_header_data2);
  boost::shared_array<uint8_t> buffer(new uint8_t[total_len]);

  char* ptr = (char*)buffer.get();
  *(uint32_t*)ptr = sizeof(g_header_data1);
  ptr += 4;
  memcpy(ptr, g_header_data1, sizeof(g_header_data1));
  ptr += sizeof(g_header_data1);

  *(uint32_t*)ptr = sizeof(g_header_data2);
  ptr += 4;
  memcpy(ptr, g_header_data2, sizeof(g_header_data2));
  ptr += sizeof(g_header_data2);

  std::string error_msg;
  ASSERT_TRUE(header.parse(buffer, total_len, error_msg));

  std::string val;
  ASSERT_TRUE(header.getValue("A", val));
  ASSERT_STREQ(val.c_str(), "B");

  ASSERT_TRUE(header.getValue("AAAAAAAAAAAAA", val));
  ASSERT_STREQ(val.c_str(), "BBBBBBB\nBBBBBBB");
}

TEST(Header, write)
{
  M_string map;
  map["haha"] = "hoho";
  map["fasdf"] = "aaaaaaaaaaaaaaa";
  map["02490\n254"] = "idsjowiejf\nioajfoiwje";

  boost::shared_array<uint8_t> buffer;
  uint32_t len;
  Header::write(map, buffer, len);

  ASSERT_EQ(len, 4*3 + 3 + strlen("haha") + strlen("hoho") + strlen("fasdf") + strlen("aaaaaaaaaaaaaaa") + strlen("02490\n254") + strlen("idsjowiejf\nioajfoiwje"));

  Header header;
  std::string error_msg;
  ASSERT_TRUE(header.parse(buffer, len, error_msg));

  std::string val;
  ASSERT_TRUE(header.getValue("haha", val));
  ASSERT_STREQ(val.c_str(), "hoho");

  ASSERT_TRUE(header.getValue("fasdf", val));
  ASSERT_STREQ(val.c_str(), "aaaaaaaaaaaaaaa");

  ASSERT_TRUE(header.getValue("02490\n254", val));
  ASSERT_STREQ(val.c_str(), "idsjowiejf\nioajfoiwje");
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


