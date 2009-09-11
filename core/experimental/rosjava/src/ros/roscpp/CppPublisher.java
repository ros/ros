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

import ros.Publisher;
import ros.RosException;
import ros.communication.Message;

public class CppPublisher<M extends Message> implements Publisher<M> {
	private String topic;
	private M msgTemplate;
	private long cppPublisher;
	
	private CppPublisher() {}
	
	protected static <M extends Message> CppPublisher<M> create(long cppHandle, String topic, M msgTemplate, int queueSize, boolean latch) throws RosException {
		CppPublisher<M> that = new CppPublisher<M>();
		that.topic = topic;
		that.msgTemplate = msgTemplate;
		that.cppPublisher = JNI.advertise(cppHandle, topic, msgTemplate, queueSize, latch);
		if (that.cppPublisher == 0) throw new RosException("Could not advertise topic " + topic);
		return that;
	}
	
	public String getTopic() { return topic; }

	public int getNumSubscribers() {
		return JNI.getNumSubscribers(cppPublisher);
	}

	public void publish(M m) {
		if (cppPublisher == 0) throw new RuntimeException("This publication was already shutdown");
		if (m.getClass() != msgTemplate.getClass()) throw new RuntimeException("This message does not match the advertised type.");
		JNI.publish(cppPublisher, m);
	}

	public boolean isValid() { return (cppPublisher != 0) && JNI.isPublisherValid(cppPublisher);} 

	public void shutdown() {
		JNI.shutdownPublisher(cppPublisher);
		cppPublisher = 0;
	}
}
