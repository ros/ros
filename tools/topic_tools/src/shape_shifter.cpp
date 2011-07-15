/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
********************************************************************/

#include <topic_tools/shape_shifter.h>

using namespace topic_tools;

bool ShapeShifter::uses_old_API_ = false;

ShapeShifter::ShapeShifter()
  :  typed(false),
     msgBuf(NULL),
     msgBufUsed(0),
     msgBufAlloc(0)
{ 
}


ShapeShifter::~ShapeShifter()
{
  if (msgBuf)
    delete[] msgBuf;
  
  msgBuf = NULL;
  msgBufAlloc = 0;
}


std::string const& ShapeShifter::getDataType()          const { return datatype; }


std::string const& ShapeShifter::getMD5Sum()            const { return md5;   }


std::string const& ShapeShifter::getMessageDefinition() const { return msg_def;  }


void ShapeShifter::morph(const std::string& _md5sum, const std::string& _datatype, const std::string& _msg_def,
                         const std::string& _latching)
{
  md5 = _md5sum;
  datatype = _datatype;
  msg_def = _msg_def;
  latching = _latching;
  typed = md5 != "*";
}


ros::Publisher ShapeShifter::advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size_, bool latch, const ros::SubscriberStatusCallback &connect_cb) const
{
  ros::AdvertiseOptions opts(topic, queue_size_, getMD5Sum(), getDataType(), getMessageDefinition(), connect_cb);
  opts.latch = latch;

  return nh.advertise(opts);
}


uint32_t ShapeShifter::size() const
{
  return msgBufUsed;
}

