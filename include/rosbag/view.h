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

#ifndef ROSBAG_VIEW_H
#define ROSBAG_VIEW_H

#include "rosbag/message_instance.h"

#include <boost/iterator/iterator_facade.hpp>

namespace rosbag
{

class View
{
    friend class Bag;

public:
    class iterator : public boost::iterator_facade<iterator, MessageInfo const, boost::forward_traversal_tag>
    {
    public:
        iterator();
        iterator(std::vector<MessageInfo>::const_iterator p);
        iterator(iterator const& other);

    private:
        friend class boost::iterator_core_access;

        bool equal(iterator const& other) const;

        void increment();

        // This wouldn't have to be const if we weren't storing a const internally
        MessageInfo const& dereference() const;

        std::vector<MessageInfo>::const_iterator pos_;
    };

    typedef iterator const_iterator;

    iterator begin() const;
    iterator end()   const;
    uint32_t size()  const;

protected:
    View(std::vector<MessageInfo> const& messages);

private:
    std::vector<MessageInfo> const messages_;
};

}

#endif
