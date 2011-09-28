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

#ifndef ROSCPP_PUBLISHER_HANDLE_H
#define ROSCPP_PUBLISHER_HANDLE_H

#include "ros/forwards.h"
#include "ros/common.h"
#include "ros/message.h"
#include "ros/serialization.h"
#include <boost/bind.hpp>

namespace ros
{
  /**
   * \brief Manages an advertisement on a specific topic.
   *
   * A Publisher should always be created through a call to NodeHandle::advertise(), or copied from one
   * that was. Once all copies of a specific
   * Publisher go out of scope, any subscriber status callbacks associated with that handle will stop
   * being called.  Once all Publishers for a given topic go out of scope the topic will be unadvertised.
   */
  class ROSCPP_DECL Publisher
  {
  public:
    Publisher() {}
    Publisher(const Publisher& rhs);
    ~Publisher();

    /**
     * \brief Publish a message on the topic associated with this Publisher.
     *
     * This version of publish will allow fast intra-process message-passing in the future,
     * so you may not mutate the message after it has been passed in here (since it will be
     * passed directly into a callback function)
     *
     */
    template <typename M>
      void publish(const boost::shared_ptr<M>& message) const
    {
      using namespace serialization;

      if (!impl_)
        {
          ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher");
          return;
        }

      if (!impl_->isValid())
        {
          ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher (topic [%s])", impl_->topic_.c_str());
          return;
        }

      ROS_ASSERT_MSG(impl_->md5sum_ != "*" || impl_->md5sum_ == mt::md5sum<M>(*message), 
                     "Trying to publish message of type [%s/%s] on a publisher with type [%s/%s]",
                     impl_->datatype_.c_str(), impl_->md5sum_.c_str(), 
                     mt::datatype<M>(*message), mt::md5sum<M>(*message));

      SerializedMessage m;
      m.type_info = &typeid(M);
      m.message = message;

      publish(boost::bind(serializeMessage<M>, boost::ref(*message)), m);
    }

    /**
     * \brief Publish a message on the topic associated with this Publisher.
     */
    template <typename M>
      void publish(const M& message) const
    {
      using namespace serialization;
      namespace mt = ros::message_traits;

      if (!impl_)
        {
          ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher");
          return;
        }

      if (!impl_->isValid())
        {
          ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher (topic [%s])", impl_->topic_.c_str());
          return;
        }

      ROS_ASSERT_MSG(impl_->md5sum_ != "*" || impl_->md5sum_ == mt::md5sum<M>(message), 
                     "Trying to publish message of type [%s/%s] on a publisher with type [%s/%s]",
                     impl_->datatype_.c_str(), impl_->md5sum_.c_str(), 
                     mt::datatype<M>(message), mt::md5sum<M>(message));

      SerializedMessage m;
      publish(boost::bind(serializeMessage<M>, boost::ref(message)), m);
    }

    /**
     * \brief Shutdown the advertisement associated with this Publisher
     *
     * This method usually does not need to be explicitly called, as automatic shutdown happens when
     * all copies of this Publisher go out of scope
     *
     * This method overrides the automatic reference counted unadvertise, and does so immediately.
     * \note Note that if multiple advertisements were made through NodeHandle::advertise(), this will
     * only remove the one associated with this Publisher
     */
    void shutdown();

    /**
     * \brief Returns the topic that this Publisher will publish on.
     */
    std::string getTopic() const;

    /**
     * \brief Returns the number of subscribers that are currently connected to this Publisher
     */
    uint32_t getNumSubscribers() const;

    /**
     * \brief Returns whether or not this topic is latched
     */
    bool isLatched() const;

    operator void*() const { return (impl_ && impl_->isValid()) ? (void*)1 : (void*)0; }

    bool operator<(const Publisher& rhs) const
    {
      return impl_ < rhs.impl_;
    }

    bool operator==(const Publisher& rhs) const
    {
      return impl_ == rhs.impl_;
    }

    bool operator!=(const Publisher& rhs) const
    {
      return impl_ != rhs.impl_;
    }

  private:

    Publisher(const std::string& topic, const std::string& md5sum, 
              const std::string& datatype, const NodeHandle& node_handle, 
              const SubscriberCallbacksPtr& callbacks);

    void publish(const boost::function<SerializedMessage(void)>& serfunc, SerializedMessage& m) const;
    void incrementSequence() const;

    class ROSCPP_DECL Impl
    {
    public:
      Impl();
      ~Impl();

      void unadvertise();
      bool isValid() const;

      std::string topic_;
      std::string md5sum_;
      std::string datatype_;
      NodeHandlePtr node_handle_;
      SubscriberCallbacksPtr callbacks_;
      bool unadvertised_;
    };
    typedef boost::shared_ptr<Impl> ImplPtr;
    typedef boost::weak_ptr<Impl> ImplWPtr;

    ImplPtr impl_;

    friend class NodeHandle;
    friend class NodeHandleBackingCollection;
  };

  typedef std::vector<Publisher> V_Publisher;
}

#endif // ROSCPP_PUBLISHER_HANDLE_H

