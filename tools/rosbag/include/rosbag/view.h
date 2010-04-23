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

#include "rosbag/message_instance.h"
#include "rosbag/query.h"

namespace rosbag {

class MessageRange;
class IndexEntry;
class ViewIterHelper;

class View
{
    friend class Bag;

public:

    //! An iterator that points to a MessageInstance from a bag
    /*!
     * This iterator derefences to a MessageInstance by VALUE, since
     * there does not actually exist a structure of MessageInstance
     * from which to return a reference.
     */
    class iterator : public boost::iterator_facade<iterator,
                                                   MessageInstance,
                                                   boost::forward_traversal_tag,
                                                   MessageInstance>
    {
    protected:
        iterator(View* view, bool end = false);

    private:
        friend class View;
        friend class boost::iterator_core_access;

		void populate();
		void populateSeek(std::multiset<IndexEntry>::const_iterator iter);

        bool equal(iterator const& other) const;

        void increment();

        MessageInstance dereference() const;

    private:
        View* view_;
        std::vector<ViewIterHelper> iters_;
        uint32_t view_revision_;
    };

    //! Typedef to const_iterator
    /*!
     * QUESTION: Is this ok to do?  The const_iterator is necessary in
     * some places, and it can't actually be used to modify the
     * underlying structure.
     */
    typedef iterator const_iterator;

    View();
    ~View();

    iterator begin();
    iterator end();
    uint32_t size()  const;

    //! Add a query to a view
    /*!
     * param bag    The bag file on which to run this query.
     * param query  The actual query to evaluate which topics to include
     */
    void addQuery(Bag& bag, Query const& query);

protected:
    void _addQuery(BagQuery* q);

    friend class iterator;

    void update();

    std::vector<MessageRange*> ranges_;
    std::vector<BagQuery*>     queries_;
    uint32_t                   view_revision_;
};

}

#endif
