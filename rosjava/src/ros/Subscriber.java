/*
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
 */

// author: Jason Wolfe



package ros;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import ros.communication.Message;

public interface Subscriber<M extends Message> {
	public String getTopic();
	public void shutdown();
	public boolean isValid();
	
	public static interface Callback<M extends Message> {
		// request must be treated as constant, since it may be concurrently accessed by other threads..
		public void call(M request);
	}
	
	public static class QueueingCallback<M extends Message> implements Callback<M> {
		private Queue<M> queue;
		
		public QueueingCallback() {
			queue = new ConcurrentLinkedQueue<M>();
		}
		
		@SuppressWarnings({ "unchecked"})
		public void call(M request) {
			synchronized(queue) {
				queue.add((M) request.clone());
				queue.notify();
			}
		}
		
		public M pop() throws InterruptedException {
			synchronized(queue) {
				while(queue.isEmpty()) {
					queue.wait();
				}
				return queue.remove();
			}
		}
		

		public M peek() throws InterruptedException {
			synchronized(queue) {
				while(queue.isEmpty()) {
					queue.wait();
				}
				return queue.peek();
			}
		}

		public void clear() {
			synchronized(queue) {
				queue.clear();
			}
		}

		
		public boolean isEmpty() {
			return queue.isEmpty();
		}
		
		public int     size() {
			return queue.size();
		}
		
	}
}
