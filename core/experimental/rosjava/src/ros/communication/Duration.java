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



package ros.communication;

public class Duration extends TimeUnit {
	public Duration() {}
	
	public Duration(int secs, int nsecs) {
		this.secs = secs;
		this.nsecs = nsecs;
		normalize();
	}

	public Duration(double secs) {
		this.secs = (int) secs;
		this.nsecs = (int) ((secs - this.secs) * 1000000000);
		normalize();
	}

	public Duration(Duration t) {
		this.secs = t.secs;
		this.nsecs = t.nsecs;
	}

	public Duration add (Duration d) {
		return new Duration(secs + d.secs,nsecs + d.nsecs);
	}

	public Duration subtract (Duration d) {
		return new Duration(secs - d.secs,nsecs - d.nsecs);
	}
	
	public void sleep() throws InterruptedException {
		Thread.sleep(totalNsecs() / 1000000);
	}
	
	public boolean isLonger(Duration d) {
		normalize();
		d.normalize();
		return (secs > d.secs) || ((secs == d.secs) && nsecs > d.nsecs);
	}
	
	public static final Duration MAX_VALUE = new Duration(Integer.MAX_VALUE, 999999999);
	
	public String toString() {
		return secs + ":" + nsecs;
	}
}
