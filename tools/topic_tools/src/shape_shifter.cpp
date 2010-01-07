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

uint8_t*
ShapeShifter::serialize(uint8_t *writePtr, uint32_t) const
{
  // yack up what we stored
  memcpy(writePtr, msgBuf, msgBufUsed);
  return writePtr + msgBufUsed;
}

uint8_t*
ShapeShifter::deserialize(uint8_t *readPtr)
{
  // Set our md5sum, datatype, and definition from the connection header
  // (Defined in base class)
  md5 = (*__connection_header)["md5sum"];
  datatype = (*__connection_header)["type"];
  msg_def = (*__connection_header)["message_definition"];
  typed = true;

  // stash this message in our buffer
  if (__serialized_length > msgBufAlloc)
  {
    delete[] msgBuf;
    msgBuf = new uint8_t[__serialized_length];
    msgBufAlloc = __serialized_length;
  }
  msgBufUsed = __serialized_length;
  memcpy(msgBuf, readPtr, __serialized_length);
    
  return NULL;
}

ros::Publisher ShapeShifter::advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size_, bool latch) const
{
  ros::AdvertiseOptions opts(topic, queue_size_, __getMD5Sum(), __getDataType(), __getMessageDefinition());
  opts.latch = latch;

  return nh.advertise(opts);
}

