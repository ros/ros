/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include <gtest/gtest.h>
#include <string>
#include "rostest/permuter.h"

using namespace rostest;


double epsilon = 1e-9;

TEST(Permuter, PermuteOption)
{
  std::vector<double> vals;
  vals.push_back(1.0);
  vals.push_back(2.0);
  vals.push_back(3.0);
  vals.push_back(4.0);
  double value = 0;
  
  PermuteOption<double> op(vals, &value);

  for ( unsigned int i = 0; i < vals.size(); i++)
  {
    EXPECT_NEAR(vals[i], value, epsilon);
    if (i < vals.size() -1)
      EXPECT_TRUE(op.step());
    else
      EXPECT_FALSE(op.step());
  };
  
  
}


TEST(Permuter, OneDoublePermuteOption)
{
  double epsilon = 1e-9;
  rostest::Permuter permuter;
  std::vector<double> vals;
  vals.push_back(1.0);
  vals.push_back(2.0);
  vals.push_back(3.0);
  vals.push_back(4.0);


  double value = 0;

  permuter.addOptionSet(vals, &value);

  for ( unsigned int i = 0; i < vals.size(); i++)
  {
    EXPECT_NEAR(vals[i], value, epsilon);
    if (i < vals.size() -1)
      EXPECT_TRUE(permuter.step());
    else
      EXPECT_FALSE(permuter.step());
  };

}

TEST(Permuter, TwoDoubleOptions)
{
  double epsilon = 1e-9;
  Permuter permuter;
  std::vector<double> vals;
  vals.push_back(1.0);
  vals.push_back(2.0);
  vals.push_back(3.0);
  vals.push_back(4.0);


  double value = 0;
  
  std::vector<double> vals2;
  vals2.push_back(9.0);
  vals2.push_back(8.0);
  vals2.push_back(7.0);
  vals2.push_back(6.0);

  double value2;

  permuter.addOptionSet(vals, &value);
  permuter.addOptionSet(vals2, &value2);
  for ( unsigned int j = 0; j < vals2.size(); j++)
    for ( unsigned int i = 0; i < vals.size(); i++)
    {
      //printf("%f?=%f %f?=%f\n", value, vals[i], value2, vals2[j]); 
      EXPECT_NEAR(vals[i], value, epsilon);
      EXPECT_NEAR(vals2[j], value2, epsilon);
      if (i == vals.size() -1 && j == vals2.size() -1)
        EXPECT_FALSE(permuter.step());
      else
        EXPECT_TRUE(permuter.step());
    };

}
TEST(Permuter, ThreeDoubleOptions)
{
  double epsilon = 1e-9;
  Permuter permuter;
  std::vector<double> vals;
  vals.push_back(1.0);
  vals.push_back(2.0);
  vals.push_back(3.0);
  vals.push_back(4.0);


  double value = 0;
  
  std::vector<double> vals2;
  vals2.push_back(9.0);
  vals2.push_back(8.0);
  vals2.push_back(7.0);
  vals2.push_back(6.0);

  double value2;

  std::vector<double> vals3;
  vals3.push_back(99.0);
  vals3.push_back(88.0);
  vals3.push_back(78.0);
  vals3.push_back(63.0);

  double value3;

  permuter.addOptionSet(vals, &value);
  permuter.addOptionSet(vals2, &value2);
  permuter.addOptionSet(vals3, &value3);

  for ( unsigned int k = 0; k < vals3.size(); k++)
    for ( unsigned int j = 0; j < vals2.size(); j++)
      for ( unsigned int i = 0; i < vals.size(); i++)
      {
        EXPECT_NEAR(vals[i], value, epsilon);
        EXPECT_NEAR(vals2[j], value2, epsilon);
        EXPECT_NEAR(vals3[k], value3, epsilon);
        if (i == vals.size() -1 && j == vals2.size() -1&& k == vals3.size() -1)
          EXPECT_FALSE(permuter.step());
        else
          EXPECT_TRUE(permuter.step());
      };
  
}

TEST(Permuter, DoubleStringPermuteOptions)
{
  double epsilon = 1e-9;
  Permuter permuter;
  std::vector<double> vals;
  vals.push_back(1.0);
  vals.push_back(2.0);
  vals.push_back(3.0);
  vals.push_back(4.0);


  double value = 0;
  
  std::vector<std::string> vals2;
  vals2.push_back("hi");
  vals2.push_back("there");
  vals2.push_back("this");
  vals2.push_back("works");

  std::string value2;

  permuter.addOptionSet(vals, &value);
  permuter.addOptionSet(vals2, &value2);

  for ( unsigned int j = 0; j < vals2.size(); j++)
    for ( unsigned int i = 0; i < vals.size(); i++)
    {
      //printf("%f?=%f %s?=%s\n", value, vals[i], value2.c_str(), vals2[j].c_str()); 
      EXPECT_NEAR(vals[i], value, epsilon);
      EXPECT_STREQ(vals2[j].c_str(), value2.c_str());
      if (i == vals.size() -1 && j == vals2.size() -1)
        EXPECT_FALSE(permuter.step());
      else
        EXPECT_TRUE(permuter.step());
    };

}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
