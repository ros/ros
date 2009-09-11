// copyright 2009 morgan quigley, bsd license blah blah

#ifndef TOPIC_TOOLS_SHAPE_SHIFTER_H
#define TOPIC_TOOLS_SHAPE_SHIFTER_H

#include "ros/ros.h"
#include <vector>
#include <string>
#include <string.h>

namespace topic_tools
{
 
// as on star trek, you've always got to be on the lookout for shape shifters
class ShapeShifter : public ros::Message
{
public:
  typedef boost::shared_ptr<ShapeShifter> Ptr;
  typedef boost::shared_ptr<ShapeShifter const> ConstPtr;

  static std::string md5, datatype, msg_def;
  static bool typed;
  static std::vector<ShapeShifter *> in_msgs;

  uint8_t *msgBuf;
  uint32_t msgBufUsed, msgBufAlloc;
  std::string topic;
  
  ShapeShifter()
  : ros::Message(), msgBuf(NULL), msgBufUsed(0), msgBufAlloc(0) { }
  virtual ~ShapeShifter() { if (msgBuf) delete[] msgBuf;
                            msgBuf = NULL; msgBufAlloc = 0; }
  virtual const std::string __getDataType() const { return datatype; }
  virtual const std::string __getMD5Sum()   const { return md5; }
  virtual const std::string __getMessageDefinition()   const { return msg_def; }
  static const std::string __s_getDataType() { return datatype; }
  static const std::string __s_getMD5Sum()   { return md5; }
  static const std::string __s_getMessageDefinition()   { return msg_def; }
  uint32_t serializationLength() const { return msgBufUsed; }
  virtual uint8_t *serialize(uint8_t *writePtr, uint32_t) const
  {
    // yack up what we stored
    memcpy(writePtr, msgBuf, msgBufUsed);
    return writePtr + msgBufUsed;
  }
  virtual uint8_t *deserialize(uint8_t *readPtr)
  {
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
};


}

#endif

