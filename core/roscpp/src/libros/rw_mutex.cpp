/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#include "ros/rw_mutex.h"
#include <apr_thread_rwlock.h>
#include <apr_pools.h>

#include <ros/assert.h>
#include <boost/thread/tss.hpp>

namespace ros
{

#define CHECK_APR_STATUS(x) \
  { \
    apr_status_t status = x; \
    ROS_ASSERT(status == APR_SUCCESS); \
  }

class APRPool
{
public:
  APRPool()
  {
    CHECK_APR_STATUS(apr_pool_create_unmanaged(&pool));
  }
  ~APRPool()
  {
    apr_pool_destroy(pool);
  }

  apr_pool_t* pool;
};
boost::thread_specific_ptr<APRPool> g_wrmutex_pool;

void createPoolIfNecessary()
{
  if (!g_wrmutex_pool.get())
  {
    g_wrmutex_pool.reset(new APRPool);
  }
}

RWMutex::RWMutex()
{
  createPoolIfNecessary();
  CHECK_APR_STATUS(apr_thread_rwlock_create(&lock_, g_wrmutex_pool->pool));
}

RWMutex::~RWMutex()
{
  CHECK_APR_STATUS(apr_thread_rwlock_destroy(lock_));
}

void RWMutex::readLock()
{
  CHECK_APR_STATUS(apr_thread_rwlock_rdlock(lock_));
}

bool RWMutex::readTryLock()
{
  apr_status_t status = apr_thread_rwlock_tryrdlock(lock_);

  if (status == APR_SUCCESS)
  {
    return true;
  }
  else if (APR_STATUS_IS_EBUSY(status))
  {
    return false;
  }
  else
  {
    ROS_BREAK();
  }

  return false;
}

void RWMutex::readUnlock()
{
  CHECK_APR_STATUS(apr_thread_rwlock_unlock(lock_));
}

void RWMutex::writeLock()
{
  CHECK_APR_STATUS(apr_thread_rwlock_wrlock(lock_));
}

bool RWMutex::writeTryLock()
{
  apr_status_t status = apr_thread_rwlock_trywrlock(lock_);

  if (status == APR_SUCCESS)
  {
    return true;
  }
  else if (APR_STATUS_IS_EBUSY(status))
  {
    return false;
  }
  else
  {
    ROS_BREAK();
  }

  return false;
}

void RWMutex::writeUnlock()
{
  CHECK_APR_STATUS(apr_thread_rwlock_unlock(lock_));
}

}
