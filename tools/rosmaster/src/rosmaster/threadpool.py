# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

"""
Internal threadpool library for zenmaster.

Adapted from U{http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/203871}

Added a 'marker' to tasks so that multiple tasks with the same
marker are not executed. As we are using the thread pool for i/o
tasks, the marker is set to the i/o name. This prevents a slow i/o
for gobbling up all of our threads
"""

import threading, logging, traceback
from time import sleep

class MarkedThreadPool:

    """Flexible thread pool class.  Creates a pool of threads, then
    accepts tasks that will be dispatched to the next available
    thread."""
    
    def __init__(self, numThreads):

        """Initialize the thread pool with numThreads workers."""
        
        self.__threads = []
        self.__resizeLock = threading.Condition(threading.Lock())
        self.__taskLock = threading.Condition(threading.Lock())
        self.__tasks = []
        self.__markers = set()
        self.__isJoining = False
        self.set_thread_count(numThreads)

    def set_thread_count(self, newNumThreads):

        """ External method to set the current pool size.  Acquires
        the resizing lock, then calls the internal version to do real
        work."""
        
        # Can't change the thread count if we're shutting down the pool!
        if self.__isJoining:
            return False
        
        self.__resizeLock.acquire()
        try:
            self.__set_thread_count_nolock(newNumThreads)
        finally:
            self.__resizeLock.release()
        return True

    def __set_thread_count_nolock(self, newNumThreads):
        
        """Set the current pool size, spawning or terminating threads
        if necessary.  Internal use only; assumes the resizing lock is
        held."""
        
        # If we need to grow the pool, do so
        while newNumThreads > len(self.__threads):
            newThread = ThreadPoolThread(self)
            self.__threads.append(newThread)
            newThread.start()
        # If we need to shrink the pool, do so
        while newNumThreads < len(self.__threads):
            self.__threads[0].go_away()
            del self.__threads[0]

    def get_thread_count(self):
        """@return: number of threads in the pool."""
        self.__resizeLock.acquire()
        try:
            return len(self.__threads)
        finally:
            self.__resizeLock.release()

    def queue_task(self, marker, task, args=None, taskCallback=None):

        """Insert a task into the queue.  task must be callable;
        args and taskCallback can be None."""
        
        if self.__isJoining == True:
            return False
        if not callable(task):
            return False
        
        self.__taskLock.acquire()
        try:
            self.__tasks.append((marker, task, args, taskCallback))
            return True
        finally:
            self.__taskLock.release()

    def remove_marker(self, marker):
        """Remove the marker from the currently executing tasks. Only one
        task with the given marker can be executed at a given time"""
        if marker is None:
            return
        self.__taskLock.acquire()        
        try:
            self.__markers.remove(marker)
        finally:
            self.__taskLock.release()            
    
    def get_next_task(self):

        """ Retrieve the next task from the task queue.  For use
        only by ThreadPoolThread objects contained in the pool."""
        
        self.__taskLock.acquire()
        try:
            retval = None
            for marker, task, args, callback in self.__tasks:
                # unmarked or not currently executing
                if marker is None or marker not in self.__markers:
                    retval = (marker, task, args, callback)
                    break
            if retval:
                # add the marker so we don't do any similar tasks
                self.__tasks.remove(retval)
                if marker is not None:
                    self.__markers.add(marker)
                return retval
            else:
                return (None, None, None, None)
        finally:
            self.__taskLock.release()
    
    def join_all(self, wait_for_tasks = True, wait_for_threads = True):
        """ Clear the task queue and terminate all pooled threads,
        optionally allowing the tasks and threads to finish."""
        
        # Mark the pool as joining to prevent any more task queueing
        self.__isJoining = True

        # Wait for tasks to finish
        if wait_for_tasks:
            while self.__tasks != []:
                sleep(.1)

        # Tell all the threads to quit
        self.__resizeLock.acquire()
        try:
            self.__set_thread_count_nolock(0)
            self.__isJoining = True

            # Wait until all threads have exited
            if wait_for_threads:
                for t in self.__threads:
                    t.join()
                    del t

            # Reset the pool for potential reuse
            self.__isJoining = False
        finally:
            self.__resizeLock.release()


        
class ThreadPoolThread(threading.Thread):
    """
    Pooled thread class.
    """
    
    threadSleepTime = 0.1

    def __init__(self, pool):
        """Initialize the thread and remember the pool."""
        threading.Thread.__init__(self)
        self.setDaemon(True) #don't block program exit
        self.__pool = pool
        self.__isDying = False
        
    def run(self):
        """
        Until told to quit, retrieve the next task and execute
        it, calling the callback if any.  
        """
        while self.__isDying == False:
            marker, cmd, args, callback = self.__pool.get_next_task()
            # If there's nothing to do, just sleep a bit
            if cmd is None:
                sleep(ThreadPoolThread.threadSleepTime)
            else:
                try:
                    try:
                        result = cmd(*args)
                    finally:
                        self.__pool.remove_marker(marker)
                    if callback is not None:
                        callback(result)
                except Exception as e:
                    logging.getLogger('rosmaster.threadpool').error(traceback.format_exc())
    
    def go_away(self):
        """ Exit the run loop next time through."""
        self.__isDying = True
