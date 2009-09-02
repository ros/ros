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


package ros.roscpp;

import ros.RosException;
import ros.Subscriber;
import ros.communication.Message;

public class CppSubscriber<M extends Message> implements Subscriber<M> {
	private String topic;
	private long   cppCallback;
	private long   cppSubscriber;

	private CppSubscriber() {}
	
	protected static <M extends Message> CppSubscriber<M> create(long cppHandle, String topic, M messageTemplate, Callback<M> callback, int queueSize) throws RosException {
		CppSubscriber<M> that = new CppSubscriber<M>();
		that.topic = topic;
		that.cppCallback = JNI.createSubCallback(callback, messageTemplate);
		if (that.cppCallback == 0) throw new RuntimeException("Could not create callback wrapper.");
		that.cppSubscriber = JNI.subscribe(cppHandle, topic, that.cppCallback, queueSize);
		if (that.cppSubscriber == 0) {
			JNI.deleteSubCallback(that.cppCallback);
			throw new RosException("Could not subscribe to topic " + topic);
		}
		return that;
	}
	
	public String getTopic() { return topic; }

	public boolean isValid() { return (cppSubscriber != 0) && JNI.isSubscriberValid(cppSubscriber); } 

	public void shutdown() {
		if (!isValid()) return;
		JNI.shutdownSubscriber(cppSubscriber);
		JNI.deleteSubCallback(cppCallback);

		cppSubscriber = 0;
	}
}
