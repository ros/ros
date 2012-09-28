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
 * Test generated messages
 */

#include <gtest/gtest.h>
#include "helpers.h"
#include "roscpp/ArrayOfFixedLength.h"
#include "roscpp/ArrayOfVariableLength.h"
#include "roscpp/EmbeddedFixedLength.h"
#include "roscpp/EmbeddedVariableLength.h"
#include "roscpp/FixedLength.h"
#include "roscpp/VariableLength.h"
#include "roscpp/WithHeader.h"
#include "roscpp/EmbeddedExternal.h"
#include "roscpp/WithTime.h"
#include "roscpp/WithDuration.h"
#include "roscpp/WithMemberNamedHeaderThatIsNotAHeader.h"
#include "roscpp/FixedLengthArrayOfExternal.h"
#include "roscpp/VariableLengthArrayOfExternal.h"
#include "roscpp/Constants.h"
#include "roscpp/VariableLengthStringArray.h"
#include "roscpp/FixedLengthStringArray.h"
#include "roscpp/HeaderNotFirstMember.h"

using namespace roscpp;

namespace roscpp
{
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(ArrayOfFixedLength, MyArrayOfFixedLength, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(ArrayOfVariableLength, MyArrayOfVariableLength, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(EmbeddedFixedLength, MyEmbeddedFixedLength, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(EmbeddedVariableLength, MyEmbeddedVariableLength, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(FixedLength, MyFixedLength, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(VariableLength, MyVariableLength, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(WithHeader, MyWithHeader, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(EmbeddedExternal, MyEmbeddedExternal, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(WithTime, MyWithTime, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(WithDuration, MyWithDuration, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(WithMemberNamedHeaderThatIsNotAHeader, MyWithMemberNamedHeaderThatIsNotAHeader, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(FixedLengthArrayOfExternal, MyFixedLengthArrayOfExternal, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(VariableLengthArrayOfExternal, MyVariableLengthArrayOfExternal, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(FixedLengthStringArray, MyFixedLengthStringArray, Allocator);
  ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(VariableLengthStringArray, MyVariableLengthStringArray, Allocator);
}

TEST(GeneratedMessages, traitsWithStandardMessages)
{
  EXPECT_TRUE(mt::isFixedSize<ArrayOfFixedLength>());
  EXPECT_FALSE(mt::isFixedSize<ArrayOfVariableLength>());
  EXPECT_TRUE(mt::isFixedSize<EmbeddedFixedLength>());
  EXPECT_FALSE(mt::isFixedSize<EmbeddedVariableLength>());
  EXPECT_TRUE(mt::isFixedSize<FixedLength>());
  EXPECT_FALSE(mt::isFixedSize<VariableLength>());
  EXPECT_FALSE(mt::isFixedSize<WithHeader>());
  EXPECT_TRUE(mt::isFixedSize<EmbeddedExternal>());
  EXPECT_TRUE(mt::isFixedSize<WithTime>());
  EXPECT_TRUE(mt::isFixedSize<WithDuration>());
  EXPECT_TRUE(mt::isFixedSize<WithMemberNamedHeaderThatIsNotAHeader>());
  EXPECT_FALSE(mt::isFixedSize<FixedLengthArrayOfExternal>());
  EXPECT_FALSE(mt::isFixedSize<VariableLengthArrayOfExternal>());
  EXPECT_FALSE(mt::isFixedSize<FixedLengthStringArray>());
  EXPECT_FALSE(mt::isFixedSize<VariableLengthStringArray>());

  EXPECT_FALSE(mt::hasHeader<ArrayOfFixedLength>());
  EXPECT_FALSE(mt::hasHeader<ArrayOfVariableLength>());
  EXPECT_FALSE(mt::hasHeader<EmbeddedFixedLength>());
  EXPECT_FALSE(mt::hasHeader<EmbeddedVariableLength>());
  EXPECT_FALSE(mt::hasHeader<FixedLength>());
  EXPECT_FALSE(mt::hasHeader<VariableLength>());
  EXPECT_TRUE(mt::hasHeader<WithHeader>());
  EXPECT_FALSE(mt::hasHeader<EmbeddedExternal>());
  EXPECT_FALSE(mt::hasHeader<WithTime>());
  EXPECT_FALSE(mt::hasHeader<WithDuration>());
  EXPECT_FALSE(mt::hasHeader<WithMemberNamedHeaderThatIsNotAHeader>());
  EXPECT_FALSE(mt::hasHeader<FixedLengthArrayOfExternal>());
  EXPECT_FALSE(mt::hasHeader<VariableLengthArrayOfExternal>());
  EXPECT_FALSE(mt::hasHeader<FixedLengthStringArray>());
  EXPECT_FALSE(mt::hasHeader<VariableLengthStringArray>());
  EXPECT_FALSE(mt::hasHeader<HeaderNotFirstMember>());

  EXPECT_FALSE(mt::isSimple<ArrayOfFixedLength>());
  EXPECT_FALSE(mt::isSimple<ArrayOfVariableLength>());
  EXPECT_FALSE(mt::isSimple<EmbeddedFixedLength>());
  EXPECT_FALSE(mt::isSimple<EmbeddedVariableLength>());
  EXPECT_FALSE(mt::isSimple<FixedLength>());
  EXPECT_FALSE(mt::isSimple<VariableLength>());
  EXPECT_FALSE(mt::isSimple<WithHeader>());
  EXPECT_FALSE(mt::isSimple<EmbeddedExternal>());
  EXPECT_FALSE(mt::isSimple<WithTime>());
  EXPECT_FALSE(mt::isSimple<WithDuration>());
  EXPECT_FALSE(mt::isSimple<WithMemberNamedHeaderThatIsNotAHeader>());
  EXPECT_FALSE(mt::isSimple<FixedLengthArrayOfExternal>());
  EXPECT_FALSE(mt::isSimple<VariableLengthArrayOfExternal>());
  EXPECT_FALSE(mt::isSimple<FixedLengthStringArray>());
  EXPECT_FALSE(mt::isSimple<VariableLengthStringArray>());
}

TEST(GeneratedMessages, traitsWithCustomAllocator)
{
  EXPECT_TRUE(mt::isFixedSize<MyArrayOfFixedLength>());
  EXPECT_FALSE(mt::isFixedSize<MyArrayOfVariableLength>());
  EXPECT_TRUE(mt::isFixedSize<MyEmbeddedFixedLength>());
  EXPECT_FALSE(mt::isFixedSize<MyEmbeddedVariableLength>());
  EXPECT_TRUE(mt::isFixedSize<MyFixedLength>());
  EXPECT_FALSE(mt::isFixedSize<MyVariableLength>());
  EXPECT_FALSE(mt::isFixedSize<MyWithHeader>());
  EXPECT_TRUE(mt::isFixedSize<MyEmbeddedExternal>());
  EXPECT_TRUE(mt::isFixedSize<MyWithTime>());
  EXPECT_TRUE(mt::isFixedSize<MyWithDuration>());
  EXPECT_TRUE(mt::isFixedSize<MyWithMemberNamedHeaderThatIsNotAHeader>());
  EXPECT_FALSE(mt::isFixedSize<MyFixedLengthArrayOfExternal>());
  EXPECT_FALSE(mt::isFixedSize<MyVariableLengthArrayOfExternal>());
  EXPECT_FALSE(mt::isFixedSize<MyFixedLengthStringArray>());
  EXPECT_FALSE(mt::isFixedSize<MyVariableLengthStringArray>());

  EXPECT_FALSE(mt::hasHeader<MyArrayOfFixedLength>());
  EXPECT_FALSE(mt::hasHeader<MyArrayOfVariableLength>());
  EXPECT_FALSE(mt::hasHeader<MyEmbeddedFixedLength>());
  EXPECT_FALSE(mt::hasHeader<MyEmbeddedVariableLength>());
  EXPECT_FALSE(mt::hasHeader<MyFixedLength>());
  EXPECT_FALSE(mt::hasHeader<MyVariableLength>());
  EXPECT_TRUE(mt::hasHeader<MyWithHeader>());
  EXPECT_FALSE(mt::hasHeader<MyEmbeddedExternal>());
  EXPECT_FALSE(mt::hasHeader<MyWithTime>());
  EXPECT_FALSE(mt::hasHeader<MyWithDuration>());
  EXPECT_FALSE(mt::hasHeader<MyWithMemberNamedHeaderThatIsNotAHeader>());
  EXPECT_FALSE(mt::hasHeader<MyFixedLengthArrayOfExternal>());
  EXPECT_FALSE(mt::hasHeader<MyVariableLengthArrayOfExternal>());
  EXPECT_FALSE(mt::hasHeader<MyFixedLengthStringArray>());
  EXPECT_FALSE(mt::hasHeader<MyVariableLengthStringArray>());

  EXPECT_FALSE(mt::isSimple<MyArrayOfFixedLength>());
  EXPECT_FALSE(mt::isSimple<MyArrayOfVariableLength>());
  EXPECT_FALSE(mt::isSimple<MyEmbeddedFixedLength>());
  EXPECT_FALSE(mt::isSimple<MyEmbeddedVariableLength>());
  EXPECT_FALSE(mt::isSimple<MyFixedLength>());
  EXPECT_FALSE(mt::isSimple<MyVariableLength>());
  EXPECT_FALSE(mt::isSimple<MyWithHeader>());
  EXPECT_FALSE(mt::isSimple<MyEmbeddedExternal>());
  EXPECT_FALSE(mt::isSimple<MyWithTime>());
  EXPECT_FALSE(mt::isSimple<MyWithDuration>());
  EXPECT_FALSE(mt::isSimple<MyWithMemberNamedHeaderThatIsNotAHeader>());
  EXPECT_FALSE(mt::isSimple<MyFixedLengthArrayOfExternal>());
  EXPECT_FALSE(mt::isSimple<MyVariableLengthArrayOfExternal>());
  EXPECT_FALSE(mt::isSimple<MyFixedLengthStringArray>());
  EXPECT_FALSE(mt::isSimple<MyVariableLengthStringArray>());
}

#define SERIALIZATION_COMPILATION_TEST(Type) \
  TEST(GeneratedMessages, serialization_##Type) \
  { \
    Type ser_val, deser_val; \
    serializeAndDeserialize(ser_val, deser_val); \
  }

SERIALIZATION_COMPILATION_TEST(ArrayOfFixedLength);
SERIALIZATION_COMPILATION_TEST(ArrayOfVariableLength);
SERIALIZATION_COMPILATION_TEST(EmbeddedFixedLength);
SERIALIZATION_COMPILATION_TEST(EmbeddedVariableLength);
SERIALIZATION_COMPILATION_TEST(FixedLength);
SERIALIZATION_COMPILATION_TEST(VariableLength);
SERIALIZATION_COMPILATION_TEST(WithHeader);
SERIALIZATION_COMPILATION_TEST(EmbeddedExternal);
SERIALIZATION_COMPILATION_TEST(WithTime);
SERIALIZATION_COMPILATION_TEST(WithDuration);
SERIALIZATION_COMPILATION_TEST(WithMemberNamedHeaderThatIsNotAHeader);
SERIALIZATION_COMPILATION_TEST(FixedLengthArrayOfExternal);
SERIALIZATION_COMPILATION_TEST(VariableLengthArrayOfExternal);
SERIALIZATION_COMPILATION_TEST(FixedLengthStringArray);
SERIALIZATION_COMPILATION_TEST(VariableLengthStringArray);

SERIALIZATION_COMPILATION_TEST(MyArrayOfFixedLength);
SERIALIZATION_COMPILATION_TEST(MyArrayOfVariableLength);
SERIALIZATION_COMPILATION_TEST(MyEmbeddedFixedLength);
SERIALIZATION_COMPILATION_TEST(MyEmbeddedVariableLength);
SERIALIZATION_COMPILATION_TEST(MyFixedLength);
SERIALIZATION_COMPILATION_TEST(MyVariableLength);
SERIALIZATION_COMPILATION_TEST(MyWithHeader);
SERIALIZATION_COMPILATION_TEST(MyEmbeddedExternal);
SERIALIZATION_COMPILATION_TEST(MyWithTime);
SERIALIZATION_COMPILATION_TEST(MyWithDuration);
SERIALIZATION_COMPILATION_TEST(MyWithMemberNamedHeaderThatIsNotAHeader);
SERIALIZATION_COMPILATION_TEST(MyFixedLengthArrayOfExternal);
SERIALIZATION_COMPILATION_TEST(MyVariableLengthArrayOfExternal);
SERIALIZATION_COMPILATION_TEST(MyFixedLengthStringArray);
SERIALIZATION_COMPILATION_TEST(MyVariableLengthStringArray);

#define ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(Type, Allocator) \
  TEST(GeneratedMessages, allocationConstructor_##Type) \
  { \
    Allocator a; \
    Type val(a); \
  }

ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(ArrayOfFixedLength, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(ArrayOfVariableLength, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(EmbeddedFixedLength, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(EmbeddedVariableLength, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(FixedLength, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(VariableLength, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(WithHeader, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(EmbeddedExternal, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(WithTime, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(WithDuration, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(WithMemberNamedHeaderThatIsNotAHeader, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(FixedLengthArrayOfExternal, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(VariableLengthArrayOfExternal, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(FixedLengthStringArray, std::allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(VariableLengthStringArray, std::allocator<void>);

ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyArrayOfFixedLength, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyArrayOfVariableLength, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyEmbeddedFixedLength, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyEmbeddedVariableLength, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyFixedLength, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyVariableLength, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyWithHeader, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyEmbeddedExternal, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyWithTime, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyWithDuration, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyWithMemberNamedHeaderThatIsNotAHeader, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyFixedLengthArrayOfExternal, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyVariableLengthArrayOfExternal, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyFixedLengthStringArray, Allocator<void>);
ALLOCATOR_CONSTRUCTOR_COMPILATION_TEST(MyVariableLengthStringArray, Allocator<void>);

TEST(Generated, serializationOStreamOperator)
{
  using namespace ros::serialization;
  Array b(new uint8_t[8]);
  OStream ostream(b.get(), 8);
  FixedLength m;
  ostream << m;
  ASSERT_EQ(ostream.getLength(), 0UL);
}

TEST(Generated, constants)
{
  EXPECT_EQ(Constants::a, 1U);
  EXPECT_EQ(Constants::b, 2);
  EXPECT_EQ(Constants::c, 3U);
  EXPECT_EQ(Constants::d, 4);
  EXPECT_EQ(Constants::e, 5U);
  EXPECT_EQ(Constants::f, 6);
  EXPECT_EQ(Constants::g, 7U);
  EXPECT_EQ(Constants::h, 8);
  EXPECT_FLOAT_EQ(Constants::fa, 1.5);
  EXPECT_FLOAT_EQ(Constants::fb, 40.9);
  EXPECT_STREQ(Constants::str.c_str(), "hello there");
  EXPECT_STREQ(Constants::str2.c_str(), "this string has \"quotes\" and \\slashes\\ in it");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



