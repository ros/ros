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
 * Test serialization templates
 */

#include <gtest/gtest.h>
#include <ros/static_assert.h>
#include <std_msgs/Header.h>
#include "helpers.h"

using namespace ros;
using namespace ros::serialization;
using namespace roscpp;

ROS_STATIC_ASSERT(sizeof(ros::Time) == 8);
ROS_STATIC_ASSERT(sizeof(ros::Duration) == 8);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Tests for compilation/validity of serialization/deserialization of primitive types
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PRIMITIVE_SERIALIZATION_TEST(Type, SerInit, DeserInit) \
  TEST(Serialization, Type) \
  { \
    Type ser_val SerInit; \
    Type deser_val DeserInit; \
    Array b = serializeAndDeserialize(ser_val, deser_val); \
    EXPECT_EQ(*(Type*)b.get(), ser_val); \
    EXPECT_EQ(ser_val, deser_val); \
  }

PRIMITIVE_SERIALIZATION_TEST(uint8_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(int8_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(uint16_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(int16_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(uint32_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(int32_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(uint64_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(int64_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(float, (5.0f), (0.0f));
PRIMITIVE_SERIALIZATION_TEST(double, (5.0), (0.0));
PRIMITIVE_SERIALIZATION_TEST(Time, (500, 10000), (0, 0));
PRIMITIVE_SERIALIZATION_TEST(Duration, (500, 10000), (0, 0));

#define PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(Type, Start, Increment) \
  TEST(Serialization, variableLengthArray_##Type) \
  { \
    std::vector<Type> ser_val, deser_val; \
    Type val = Start; \
    for (uint32_t i = 0; i < 8; ++i) \
    { \
      ser_val.push_back(val); \
      val = val + Increment; \
    } \
    \
    Array b = serializeAndDeserialize(ser_val, deser_val); \
    EXPECT_TRUE(ser_val == deser_val); \
    \
    EXPECT_EQ(*(uint32_t*)b.get(), (uint32_t)ser_val.size()); \
    for(size_t i = 0; i < ser_val.size(); ++i) \
    { \
      Type* ptr = ((Type*)(b.get() + 4)) + i; \
      EXPECT_EQ(*ptr, ser_val[i]); \
    } \
  }

PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(uint8_t, 65, 1);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(int8_t, 65, 1);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(uint16_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(int16_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(uint32_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(int32_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(uint64_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(int64_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(float, 0.0f, 100.0f);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(double, 0.0, 100.0);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(Time, Time(), Duration(100));
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(Duration, Duration(), Duration(100));

#define PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(Type, Start, Increment) \
  TEST(Serialization, fixedLengthArray_##Type) \
  { \
    boost::array<Type, 8> ser_val, deser_val; \
    Type val = Start; \
    for (uint32_t i = 0; i < 8; ++i) \
    { \
      ser_val[i] = val; \
      val = val + Increment; \
    } \
    \
    Array b = serializeAndDeserialize(ser_val, deser_val); \
    EXPECT_TRUE(ser_val == deser_val); \
    \
    for(size_t i = 0; i < ser_val.size(); ++i) \
    { \
      Type* ptr = ((Type*)b.get()) + i; \
      EXPECT_EQ(*ptr, ser_val[i]); \
    } \
  }

PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(uint8_t, 65, 1);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(int8_t, 65, 1);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(uint16_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(int16_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(uint32_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(int32_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(uint64_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(int64_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(float, 0.0f, 100.0f);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(double, 0.0, 100.0);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(Time, Time(), Duration(100));
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(Duration, Duration(), Duration(100));

TEST(Serialization, string)
{
  std::string ser_val = "hello world";
  std::string deser_val;
  Array b = serializeAndDeserialize(ser_val, deser_val);
  EXPECT_STREQ(ser_val.c_str(), deser_val.c_str());

  EXPECT_EQ(*(uint32_t*)b.get(), (uint32_t)ser_val.size());
  EXPECT_EQ(memcmp(b.get() + 4, ser_val.data(), ser_val.size()), 0);
}

TEST(Serialization, variableLengthArray_string)
{
  std::vector<std::string> ser_val, deser_val;
  ser_val.push_back("hello world");
  ser_val.push_back("hello world22");
  ser_val.push_back("hello world333");
  ser_val.push_back("hello world4444");
  ser_val.push_back("hello world55555");
  Array b = serializeAndDeserialize(ser_val, deser_val);
  EXPECT_TRUE(ser_val == deser_val);
}

TEST(Serialization, fixedLengthArray_string)
{
  boost::array<std::string, 5> ser_val, deser_val;
  ser_val[0] = "hello world";
  ser_val[1] = "hello world22";
  ser_val[2] = "hello world333";
  ser_val[3] = "hello world4444";
  ser_val[4] = "hello world55555";
  Array b = serializeAndDeserialize(ser_val, deser_val);
  EXPECT_TRUE(ser_val == deser_val);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Test custom types and traits
////////////////////////////////////////////////////////////////////////////////////////////

// Class used to make sure fixed-size simple structs use a memcpy when serializing an array of them
// serialization only serializes a, memcpy will get both a and b.
struct FixedSizeSimple
{
  FixedSizeSimple()
  : a(0)
  , b(0)
  {}

  int32_t a;
  int32_t b;
};

namespace ros
{
namespace message_traits
{
template<> struct IsFixedSize<FixedSizeSimple> : public TrueType {};
template<> struct IsSimple<FixedSizeSimple> : public TrueType {};
} // namespace message_traits

namespace serialization
{
template<>
struct Serializer<FixedSizeSimple>
{
  template<typename Stream>
  inline static void write(Stream& stream, const FixedSizeSimple& v)
  {
    serialize(stream, v.a);
  }

  template<typename Stream>
  inline static void read(Stream& stream, FixedSizeSimple& v)
  {
    deserialize(stream, v.a);
  }

  inline static uint32_t serializedLength(const FixedSizeSimple& v)
  {
    return 4;
  }
};
} // namespace serialization
} // namespace ros

TEST(Serialization, fixedSizeSimple_vector)
{
  {
    FixedSizeSimple in, out;
    in.a = 1;
    in.b = 1;

    serializeAndDeserialize(in, out);
    ASSERT_EQ(out.a, 1);
    ASSERT_EQ(out.b, 0);
  }

  {
    std::vector<FixedSizeSimple> in, out;
    in.resize(1);
    in[0].a = 1;
    in[0].b = 1;

    serializeAndDeserialize(in, out);
    ASSERT_EQ(out[0].a, 1);
    ASSERT_EQ(out[0].b, 1);
  }
}

TEST(Serialization, fixedSizeSimple_array)
{
  boost::array<FixedSizeSimple, 2> in, out;
  in[0].a = 1;
  in[0].b = 1;

  serializeAndDeserialize(in, out);
  ASSERT_EQ(out[0].a, 1);
  ASSERT_EQ(out[0].b, 1);
}

// Class used to make sure fixed-size non-simple structs only query the length of the first element
// in an array.
struct FixedSizeNonSimple
{
  FixedSizeNonSimple()
  : length_to_report(4)
  {}

  int32_t length_to_report;
};

namespace ros
{
namespace message_traits
{
template<> struct IsFixedSize<FixedSizeNonSimple> : public TrueType {};
} // namespace message_traits

namespace serialization
{
template<>
struct Serializer<FixedSizeNonSimple>
{
  inline static uint32_t serializedLength(const FixedSizeNonSimple& v)
  {
    return v.length_to_report;
  }
};
} // namespace serialization
} // namespace ros

TEST(Serialization, fixedSizeNonSimple_vector)
{
  std::vector<FixedSizeNonSimple> in;
  in.resize(2);
  in[1].length_to_report = 100;

  int32_t len = ros::serialization::serializationLength(in);
  ASSERT_EQ(len, 12);  // 12 = 4 bytes for each item + 4-byte array length
}

TEST(Serialization, fixedSizeNonSimple_array)
{
  boost::array<FixedSizeNonSimple, 2> in;
  in[1].length_to_report = 100;

  int32_t len = ros::serialization::serializationLength(in);
  ASSERT_EQ(len, 8);  // 8 = 4 bytes for each item
}

// Class used to make sure variable-size structs query the length of the every element
// in an array.
struct VariableSize
{
  VariableSize()
  : length_to_report(4)
  {}

  int32_t length_to_report;
};

namespace ros
{
namespace serialization
{
template<>
struct Serializer<VariableSize>
{
  inline static uint32_t serializedLength(const VariableSize& v)
  {
    return v.length_to_report;
  }
};
} // namespace serialization
} // namespace ros

TEST(Serialization, variableSize_vector)
{
  std::vector<VariableSize> in;
  in.resize(2);
  in[1].length_to_report = 100;

  int32_t len = ros::serialization::serializationLength(in);
  ASSERT_EQ(len, 108);  // 108 = 4 bytes for the first item + 100 bytes for the second + 4-byte array length
}

TEST(Serialization, variableSize_array)
{
  boost::array<VariableSize, 2> in;
  in[1].length_to_report = 100;

  int32_t len = ros::serialization::serializationLength(in);
  ASSERT_EQ(len, 104);  // 104 = 4 bytes for the first item + 100 bytes for the second
}

struct AllInOneSerializer
{
  uint32_t a;
};

namespace ros
{
namespace serialization
{
template<>
struct Serializer<AllInOneSerializer>
{
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t)
  {
    stream.next(t.a);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
};
} // namespace serialization
} // namespace ros

TEST(Serialization, allInOne)
{
  AllInOneSerializer in, out;
  in.a = 5;
  serializeAndDeserialize(in, out);
  ASSERT_EQ(out.a, in.a);
}

// Class with a header, used to ensure message_traits::header(m) returns the header
struct WithHeader
{
  WithHeader()
  {}

  std_msgs::Header header;
};

// Class without a header, used to ensure message_traits::header(m) return NULL
struct WithoutHeader
{
  WithoutHeader()
  {}
};

namespace ros
{
namespace message_traits
{
template<> struct HasHeader<WithHeader> : public TrueType {};
} // namespace message_traits
} // namespace ros

TEST(MessageTraits, headers)
{
  WithHeader wh;
  WithoutHeader woh;
  const WithHeader cwh;
  const WithoutHeader cwoh;

  wh.header.seq = 100;
  ASSERT_TRUE(ros::message_traits::header(wh) != 0);
  ASSERT_EQ(ros::message_traits::header(wh)->seq, 100UL);

  ASSERT_TRUE(ros::message_traits::header(woh) == 0);

  ASSERT_TRUE(ros::message_traits::header(cwh) != 0);
  ASSERT_TRUE(ros::message_traits::header(cwoh) == 0);

  ASSERT_TRUE(ros::message_traits::frameId(wh) != 0);
  ASSERT_TRUE(ros::message_traits::frameId(woh) == 0);
  ASSERT_TRUE(ros::message_traits::frameId(cwh) != 0);
  ASSERT_TRUE(ros::message_traits::frameId(cwoh) == 0);

  ASSERT_TRUE(ros::message_traits::timeStamp(wh) != 0);
  ASSERT_TRUE(ros::message_traits::timeStamp(woh) == 0);
  ASSERT_TRUE(ros::message_traits::timeStamp(cwh) != 0);
  ASSERT_TRUE(ros::message_traits::timeStamp(cwoh) == 0);
}

TEST(Serialization, bufferOverrun)
{
  Array b(new uint8_t[4]);
  IStream stream(b.get(), 4);
  uint32_t i;
  deserialize(stream, i);
  try
  {
    deserialize(stream, i);
    FAIL();
  }
  catch(ros::Exception&)
  {
    SUCCEED();
  }
}

TEST(Serialization, streamOperators)
{
  Array b(new uint8_t[4]);
  OStream ostream(b.get(), 4);
  uint32_t a = 5;
  ostream << a;
  a = 100;
  IStream istream(b.get(), 4);
  istream >> a;
  ASSERT_EQ(a, 5UL);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



