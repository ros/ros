/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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


/** \author Tully Foote */

#ifndef ROSTEST_PERMUTER_H
#define ROSTEST_PERMUTER_H

#include <vector>
#include "boost/thread/mutex.hpp"

namespace rostest
{
/** \brief A base class for storing pointers to generic data types
 */
class PermuteOptionBase
{
public:
  virtual void reset() =0;
  virtual bool step() =0;
  virtual ~PermuteOptionBase() {};
};


/**\brief A class to hold a set of option values and currently used state
 *  This class holds 
 */
template<class T>
class PermuteOption : public PermuteOptionBase
{
public:
  PermuteOption(const std::vector<T>& options, T* output)
  {
    options_ = options;
    output_ = output;
    reset();
  }
  
  virtual ~PermuteOption(){};

  void reset(){
    boost::mutex::scoped_lock lock(access_mutex_);
    current_element_ = options_.begin();
    *output_ = *current_element_;
  };
  
  bool step()
  {
    boost::mutex::scoped_lock lock(access_mutex_);
    current_element_++;
    if (current_element_ == options_.end())
      return false;
    *output_ = *current_element_;
    return true;
  };

private:
  /// Local storage of the possible values
  std::vector<T> options_;
  /// The output variable
  T* output_;
  typedef typename std::vector<T>::iterator V_T_iterator;
  /// The last updated element
  V_T_iterator current_element_;

  boost::mutex access_mutex_;

};

/** \brief A class to provide easy permutation of options
 * This class provides a way to collapse independent 
 * permutations of options into a single loop.  
 */
class Permuter
{
public:
  /** \brief Destructor to clean up allocated data */
  virtual ~Permuter(){ clearAll();};


  /** \brief Add a set of values and an output to the iteration
   * @param values The set of possible values for this output
   * @param output The value to set at each iteration
   */
  template<class T>
  void addOptionSet(const std::vector<T>& values, T* output)
  {
    boost::mutex::scoped_lock lock(access_mutex_);
    options_.push_back(static_cast<PermuteOptionBase*> (new PermuteOption<T>(values, output)));
    lock.unlock();//reset locks on its own
    reset();
  };


  /** \brief Reset the internal counters */
  void reset(){
    boost::mutex::scoped_lock lock(access_mutex_);
    for (unsigned int level= 0; level < options_.size(); level++)
      options_[level]->reset();
  };

  /** \brief Iterate to the next value in the iteration
   * Returns true unless done iterating.  
   */
  bool step()
  {
    boost::mutex::scoped_lock lock(access_mutex_);
    // base case just iterating
    for (unsigned int level= 0; level < options_.size(); level++)
    {
      if(options_[level]->step())
      {
        //printf("stepping level %d returning true \n", level);
        return true;
      }
      else
      {
        //printf("reseting level %d\n", level);
        options_[level]->reset();
      }
    }
    return false;
  };

  /** \brief Clear all stored data */
  void clearAll()
  {
    boost::mutex::scoped_lock lock(access_mutex_);
    for ( unsigned int i = 0 ; i < options_.size(); i++)
    {
      delete options_[i];
    }
    options_.clear();
  };

private:
  std::vector<PermuteOptionBase*> options_; ///< Store all the option objects
  boost::mutex access_mutex_;
};


}

#endif //ROSTEST_PERMUTER_H
