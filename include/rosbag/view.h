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

#include <boost/iterator/iterator_facade.hpp>

#include "rosbag/message_info.h"
#include "rosbag/query.h"

namespace rosbag {

class MessageRange;
class IndexEntry;
class ViewIterHelper;

// Our current View has a bug.  Our internal storage end is based on
// an iterator element rather than a time.  This means if we update
// the underlying index to include a new message after our end-time
// but before where our end-iterator points, our view will include
// it when it shouldn't.  Similarly, adding new messages before our
// first message but after our start time won't be capture in the
// view either.
class View
{
    friend class Bag;

public:
    class iterator : public boost::iterator_facade<iterator,
												   MessageInfo,
												   boost::forward_traversal_tag,
												   MessageInfo>
    {
    protected:
    	// NOTE: the default constructor on the merge_queue means this is an empty queue,
    	//       i.e. our definition of end
        iterator(View const* view, bool end = false);

    private:
        friend class View;
        friend class boost::iterator_core_access;

		void populate();
		void populateSeek(std::vector<IndexEntry>::const_iterator iter);

        bool equal(iterator const& other) const;

        void increment();

        MessageInfo dereference() const;

    private:
        View const* view_;
        std::vector<ViewIterHelper> iters_;
        uint32_t view_revision_;
    };

    typedef iterator const_iterator;

    View();
    ~View();

    iterator begin() const;
    iterator end()   const;
    uint32_t size()  const;

    void addQuery(Bag& bag, Query const& query);

protected:
    friend class iterator;

    std::vector<MessageRange*> ranges_;
    std::vector<BagQuery*>     queries_;
    uint32_t                   view_revision_;
};

}

#endif
