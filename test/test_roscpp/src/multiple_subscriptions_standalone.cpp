/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

/* Author: Morgan Quigley */

/*
 * Subscribe to a topic multiple times
 */

#include <string>
#include <time.h>
#include <stdlib.h>
#include <cstdio>
#include "ros/node.h"
#include <test_roscpp/TestArray.h>

int g_argc;
char** g_argv;

class MultiSub 
{
  public:
    ros::Node *n;
    test_roscpp::TestArray msg0, msg1, msg2, msg3, msg_dummy;
    bool got_it[4], should_have_it[4];
    bool test_ready;
    int n_test;

    void cb0() { if (!test_ready) return; got_it[0] = true; }
    void cb1() { if (!test_ready) return; got_it[1] = true; }
    void cb2() { if (!test_ready) return; got_it[2] = true; }
    void cb3() { if (!test_ready) return; got_it[3] = true; }
    void cb_verify()
    {
      if (!test_ready)
        return;
      n_test++;
      /*
      ASSERT_TRUE(((should_have_it[0] ? got_it[0] : true) &&
                   (should_have_it[1] ? got_it[1] : true) &&
                   (should_have_it[2] ? got_it[2] : true) &&
                   (should_have_it[3] ? got_it[3] : true)));
      */
    }
    void cb_reset()
    {
      got_it[0] = got_it[1] = got_it[2] = got_it[3] = false; test_ready = true;
    }

    MultiSub() {}
    void SetUp()
    {
      ros::init(g_argc, g_argv);
      n = new ros::Node("subscriber");
    }
    void TearDown()
    {
      
      delete n;
    }

    bool sub(int cb_num)
    { 
      printf("subscribing %d\n", cb_num);
      switch(cb_num)
      {
        case 0: return n->subscribe("test_roscpp/pubsub_test", msg0, 
                                    &MultiSub::cb0, this, 10);
        case 1: return n->subscribe("test_roscpp/pubsub_test", msg1, 
                                    &MultiSub::cb1, this, 10);
        case 2: return n->subscribe("test_roscpp/pubsub_test", msg2, 
                                    &MultiSub::cb2, this, 10);
        case 3: return n->subscribe("test_roscpp/pubsub_test", msg3, 
                                    &MultiSub::cb3, this, 10);
        default: return false;
      }
    }
    bool sub_wrappers()
    {
      printf("sub wrappers\n");
      bool ok = true;
      ok &= n->subscribe("test_roscpp/pubsub_test", msg_dummy, 
                         &MultiSub::cb_verify, this, 10);
      ok &= n->subscribe("test_roscpp/pubsub_test", msg_dummy, 
                         &MultiSub::cb_reset, this, 10);
      return ok;
    }
    bool unsub(int cb_num)
    {
      printf("unsubscribing %d\n", cb_num);
      switch(cb_num)
      {
        case 0: return n->unsubscribe("test_roscpp/pubsub_test", 
                                      &MultiSub::cb0, this);
        case 1: return n->unsubscribe("test_roscpp/pubsub_test", 
                                      &MultiSub::cb1, this);
        case 2: return n->unsubscribe("test_roscpp/pubsub_test", 
                                      &MultiSub::cb2, this);
        case 3: return n->unsubscribe("test_roscpp/pubsub_test", 
                                      &MultiSub::cb3, this);
        default: return false;
      }
    }
    bool unsub_wrappers()
    {
      printf("unsub wrappers\n");
      bool ok = true;
      ok &= n->unsubscribe("test_roscpp/pubsub_test", &MultiSub::cb_verify, this);
      ok &= n->unsubscribe("test_roscpp/pubsub_test", &MultiSub::cb_reset, this);
      return ok;
    }
    
    void run()
    {
      test_ready = false;

      for (int i = 0; i < 0x10; i++)
      {
        for (int j = 0; j < 4; j++)
          should_have_it[j] = (i & (1 << j) ? true : false);

        printf(" testing: %d, %d, %d, %d\n",
            should_have_it[0],
            should_have_it[1],
            should_have_it[2],
            should_have_it[3]);

        for (int j = 0; j < 4; j++)
          if (should_have_it[j])
            sub(j);
        sub_wrappers();

        ros::Time t_start = ros::Time::now();
        n_test = 0;
        while (n_test < 100 && 
               ros::Time::now() - t_start < ros::Duration(2.0))
        {
          static int count = 0;
          if (count++ % 10 == 0)
            printf("%d/100 tests completed...\n", n_test);
          ros::Duration(0, 10000000).sleep();
        }

        for (int j = 0; j < 4; j++)
          if (should_have_it[j])
            unsub(j);
        unsub_wrappers();
      }
    }
};

int main(int argc, char** argv)
{
//  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  MultiSub multi_sub;
  multi_sub.SetUp();
  multi_sub.run();
  multi_sub.TearDown();
  return 0;
}

