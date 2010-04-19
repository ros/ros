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

#ifndef ROSBAG_QUERY_H
#define ROSBAG_QUERY_H

#include "ros/time.h"

#include <map>
#include <vector>

namespace rosbag {

class Bag;
class IndexEntry;
class TopicInfo;

//! A base class for specifying queries from a bag
class Query
{
public:
	Query(ros::Time const& start_time = ros::TIME_MIN,
		  ros::Time const& end_time   = ros::TIME_MAX);
	virtual ~Query();

	ros::Time getStartTime() const;
	ros::Time getEndTime()   const;

	virtual bool evaluate(TopicInfo const*) const;

private:
	ros::Time start_time_;
	ros::Time end_time_;
};

//! A Query the returns messages on a specified topic
class TopicQuery : public Query
{
public:
	TopicQuery(std::vector<std::string> const& topics,
			   ros::Time const& start_time = ros::TIME_MIN,
			   ros::Time const& end_time   = ros::TIME_MAX);

	virtual bool evaluate(TopicInfo const*) const;

private:
	std::vector<std::string> topics_;
};

//! Pairs of queries and the bags they come from (used internally by View)
typedef std::pair<Bag*, Query> BagQuery;

struct MessageRange
{
	MessageRange(std::vector<IndexEntry>::const_iterator const& _begin,
				 std::vector<IndexEntry>::const_iterator const& _end,
				 TopicInfo const* _topic_info,
				 BagQuery const* _bag_query);

	std::vector<IndexEntry>::const_iterator begin;
	std::vector<IndexEntry>::const_iterator end;
	TopicInfo const* topic_info;
	BagQuery const* bag_query;           //!< pointer to vector of queries in View
};

//! The actual iterator data structure
struct ViewIterHelper
{
	ViewIterHelper(std::vector<IndexEntry>::const_iterator _iter,
				   MessageRange const* _range);

	std::vector<IndexEntry>::const_iterator iter;
	MessageRange const* range;  //!< pointer to vector of ranges in View
};

struct ViewIterHelperCompare
{
	bool operator()(ViewIterHelper const& a, ViewIterHelper const& b);
};

}

#endif
