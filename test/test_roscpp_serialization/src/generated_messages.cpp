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
#include "test_roscpp_serialization/helpers.h"
#include "test_roscpp_serialization/ArrayOfFixedLength.h"
#include "test_roscpp_serialization/ArrayOfVariableLength.h"
#include "test_roscpp_serialization/EmbeddedFixedLength.h"
#include "test_roscpp_serialization/EmbeddedVariableLength.h"
#include "test_roscpp_serialization/FixedLength.h"
#include "test_roscpp_serialization/VariableLength.h"
#include "test_roscpp_serialization/WithHeader.h"
#include "test_roscpp_serialization/EmbeddedExternal.h"
#include "test_roscpp_serialization/WithTime.h"
#include "test_roscpp_serialization/WithDuration.h"
#include "test_roscpp_serialization/WithMemberNamedHeaderThatIsNotAHeader.h"

using namespace test_roscpp_serialization;

ROS_TYPEDEF_MESSAGE_WITH_ALLOCATOR(test_roscpp_serialization, ArrayOfFixedLength, MyArrayOfFixedLength, Allocator);
ROS_TYPEDEF_MESSAGE_WITH_ALLOCATOR(test_roscpp_serialization, ArrayOfVariableLength, MyArrayOfVariableLength, Allocator);
ROS_TYPEDEF_MESSAGE_WITH_ALLOCATOR(test_roscpp_serialization, EmbeddedFixedLength, MyEmbeddedFixedLength, Allocator);
ROS_TYPEDEF_MESSAGE_WITH_ALLOCATOR(test_roscpp_serialization, EmbeddedVariableLength, MyEmbeddedVariableLength, Allocator);
ROS_TYPEDEF_MESSAGE_WITH_ALLOCATOR(test_roscpp_serialization, FixedLength, MyFixedLength, Allocator);
ROS_TYPEDEF_MESSAGE_WITH_ALLOCATOR(test_roscpp_serialization, VariableLength, MyVariableLength, Allocator);
ROS_TYPEDEF_MESSAGE_WITH_ALLOCATOR(test_roscpp_serialization, WithHeader, MyWithHeader, Allocator);
ROS_TYPEDEF_MESSAGE_WITH_ALLOCATOR(test_roscpp_serialization, EmbeddedExternal, MyEmbeddedExternal, Allocator);
ROS_TYPEDEF_MESSAGE_WITH_ALLOCATOR(test_roscpp_serialization, WithTime, MyWithTime, Allocator);
ROS_TYPEDEF_MESSAGE_WITH_ALLOCATOR(test_roscpp_serialization, WithDuration, MyWithDuration, Allocator);
ROS_TYPEDEF_MESSAGE_WITH_ALLOCATOR(test_roscpp_serialization, WithMemberNamedHeaderThatIsNotAHeader, MyWithMemberNamedHeaderThatIsNotAHeader, Allocator);

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



