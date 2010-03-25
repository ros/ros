/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*********************************************************************/

#ifndef ROSBAG_H
#define ROSBAG_H

#include "ros/header.h"
#include "ros/time.h"
#include "ros/message.h"
#include "ros/ros.h"



#include <boost/thread/mutex.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <set>

namespace rosbag
{
  typedef unsigned long long pos_t;

  struct MsgInfo
  {
    std::string msg_def;
    std::string md5sum;
    std::string datatype;
  };
  
  struct IndexEntry
  {
    uint32_t sec;
    uint32_t nsec;
    pos_t    pos;
  };


  namespace bagmode
  {
    //! Enumerated modes
    enum BagMode
      {
        Read   = 0x01, //!< Open a bag file for reading
        Write  = 0x02, //!< Open a bag file for writing
        Append = 0x04, //!< Open a bag file for appending
        Default   = Read //!< Enable "intensities" and "stamps" channels
      };
  }


  //! Base class for rosbag exceptions
  class Exception : public std::runtime_error {};

  //! Exception thrown if trying to add or read from an unopened
  //  recorder/player
  class BagNotOpenException : public Exception {};

  //! Exception thrown when assorted IO problems
  class BagIOException : public Exception {};

  //! Exception thrown if an invalid MsgPos (such as from another bag)
  //  is passed to seek
  class InvalidMsgPosException : public Exception {};

  //! Exception thrown if trying to instantiate a MsgInstance as the
  //  wrong type
  class InstantiateException : public Exception {};

  class Bag;
  class MessageInstance;

  //! Typedef for index: map of topic_name -> list of MessageInstance
  typedef std::map<std::string, std::multiset<MessageInstance> > BagIndex;

  class MessageList
  {
    friend class Bag;
  public:
    class iterator {
      friend class MessageList;
    public:
      typedef std::random_access_iterator_tag   iterator_category;
      typedef int                         difference_type;
      typedef MessageInstance             value_type;
      typedef const MessageInstance*      pointer;	
      typedef const MessageInstance&      reference;	      

      // Reference
      reference	operator*() const;
      //      {
        //        return m_pNode->m_Data;
      //      }

      // Pointer
      pointer operator->() const;
      /*
      {
        return (&**this);
      }
      */
      
      // Prefix increment
      iterator& operator++()
      {
        pos_++;
        return *this;
      }
      
      // Postfix increment
      iterator operator++(int)
      {
        iterator itTemp(*this);
        pos_++;
        if (pos_ > message_list_.size())
        {
          pos_ = message_list_.size();
        }
        return itTemp;
      }

      // Equality test
      bool operator==(const iterator& itR) const
      {
        return ( this->pos_ == itR.pos_ );
      }

    protected:
      iterator(const MessageList& message_list, const uint32_t pos) : message_list_(message_list), pos_(pos) {}
    private:
      const MessageList& message_list_;
      uint32_t pos_;
    };

    typedef iterator const_iterator;

    iterator begin() const;
    iterator end() const;

    uint32_t size() const { return count_; }
  protected:

    MessageList(const Bag& bag,
                const std::vector<std::string>& topics,
                const ros::Time& start_time,
                const ros::Time& end_time);

  private:
    
    const Bag& bag_;
    const std::vector<std::string> topics_;
    const ros::Time start_time_;
    const ros::Time end_time_;
    uint32_t count_;
  };

  //! Class for writinging to a bagfile
  class Bag
  {
  public:
     //! Constructor
    Bag();

    //! Destructor
    ~Bag();
    
    //! Open a bagfile by name 
    bool open(const std::string &file_name, int mode = bagmode::Default);
    
    //! Close bagfile 
    /*!
     *  Make sure bagfile is written out to disk, index is appended,
     *  file descriptor is closed, etc..
     */
    void close();
    
    //! Write a message into the bag file
    /*!
     * \param topic_name  The topic name
     * \param time        Timestamp of the message
     * \param msg         A pointer to the message to be added.
     *
     * Can throw BagNotOpenException or BagIOException
     */
    void write(const std::string& topic_name, ros::Time time, ros::Message::ConstPtr msg);

    //! Write a message into the bag file
    /*!
     * \param topic_name  The topic name
     * \param time        Timestamp of the message
     * \param msg         A const reference to a message
     *
     * Can throw BagNotOpenException or BagIOException
     */
    void write(const std::string& topic_name, ros::Time time, const ros::Message& msg);

    //! Write a message into the bag file
    /*!
     * \param topic_name  The topic name
     * \param time        Timestamp of the message
     * \param msg         A MessageInstance as reterned by reader
     *
     * Can throw BagNotOpenException or BagIOException
     */
    void write(const std::string& topic_name, ros::Time time, const MessageInstance& msg);

    
    MessageList getMessageListByTopic(const std::vector<std::string>& topics,
                             const ros::Time& start_time = ros::TIME_MIN, 
                             const ros::Time& end_time = ros::TIME_MAX);

    MessageList getMessageListByType(const std::vector<std::string>& types,
                             const ros::Time& start_time = ros::TIME_MIN, 
                             const ros::Time& end_time = ros::TIME_MAX);

  private:

    bool readMode()   { return mode_ & bagmode::Read; }
    bool writeMode()  { return mode_ & bagmode::Write; }
    bool appendMode() { return mode_ & bagmode::Append; }

    void         closeFile();

    const void*  getHeaderBuffer();
    unsigned int getHeaderBufferLength();
    void         resetHeaderBuffer();
    void         writeFieldToHeaderBuffer(const std::string& name, const void* value, unsigned int value_len);
    bool         checkDisk();
    
    void         writeVersion();
    void         writeFileHeader();
    void         writeIndex();
    
    void         writeRecord(const ros::M_string& fields, const char* data, uint32_t data_len);
    void         writeHeader(const ros::M_string& fields, uint32_t data_len);
    void         writefil(const char* s, std::streamsize n);
    void         writefil(const std::string& s);
    void         seek(pos_t pos);


    // Player helper variables
    int version_;
    int version_major_;
    int version_minor_;


    // Player functions
    bool readHeader(ros::Header& header, uint32_t& next_msg_size);
    bool readVersion();
    bool readFileHeader();
    bool readIndex();
    bool readDefs();
    bool readDef(uint64_t);
    ros::M_string::const_iterator checkField(const ros::M_string& fields,
                                             const std::string& field,
                                             unsigned int min_len,
                                             unsigned int max_len,
                                             bool required);

    // Other stufff... oh how this needs brutal cleaning

    int mode_;

    std::string file_name_;
    
    std::ifstream                       read_stream_;
    std::ofstream                       write_stream_;

    pos_t                               record_pos_;
    pos_t                               file_header_pos_;
    pos_t                               index_data_pos_;

    boost::mutex                        record_mutex_;
    
    // Keep track of which topics have had their message definitions written already
    std::map<std::string, MsgInfo>      topics_recorded_;
    boost::mutex                        topics_recorded_mutex_;
    
    std::map<std::string, std::vector<IndexEntry> > topic_indexes_;
    
    // Reusable buffer in which to assemble the message header before writing to file
    unsigned char* header_buf_;
    unsigned int   header_buf_len_;
    unsigned int   header_buf_size_;
    
    // Reusable buffer in which to assemble the message data before writing to file
    unsigned char* message_buf_;
    unsigned int   message_buf_len_;
    unsigned int   message_buf_size_;
    
    bool writing_enabled_;
    
    boost::mutex       check_disk_mutex_;
    ros::WallTime      check_disk_next_;
    ros::WallTime      warn_next_;

  };

  //! Class representing an actual message
  class MessageInstance
  {
  public:
    const std::string getTopic();
    const std::string getDatatype();
    const std::string getMd5sum();
    const ros::Time getTime();

    /*!
     * Templated type-check 
     */
    template <class T>
    bool isType();

    /*!
     * Templated instantiate
     */
    template <class T>
    boost::shared_ptr<T const> instantiate() const;

  protected:
    MessageInstance(const MsgInfo& info,
                    const IndexEntry& ind,
                    const Bag& bag);

  private:

    const MsgInfo& info_;
    const IndexEntry index_;
    const Bag& bag_;

  };

}



#endif
