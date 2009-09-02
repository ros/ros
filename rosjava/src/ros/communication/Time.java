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

import ros.Ros;

public class Time extends TimeUnit {
	public Time() {}
	
	public Time(int secs, int nsecs) {
		this.secs = secs;
		this.nsecs = nsecs;
		normalize();
	}

	public Time(double secs) {
		this.secs = (int) secs;
		this.nsecs = (int) ((secs - this.secs) * 1000000000);
		normalize();
	}

	public Time(Time t) {
		this.secs = t.secs;
		this.nsecs = t.nsecs;
	}
	
	public Time add (Duration d) {
		return new Time(secs + d.secs,nsecs + d.nsecs);
	}

	public Time subtract (Duration d) {
		return new Time(secs - d.secs,nsecs - d.nsecs);
	}

	public Duration subtract (Time t) {
		return new Duration(secs - t.secs,nsecs - t.nsecs);
	}

	public boolean laterThan(Time t) {
		normalize();
		t.normalize();
		return (secs > t.secs) || ((secs == t.secs) && nsecs > t.nsecs);
	}

	public static Time now() { return Ros.getInstance().now(); }
	
	public boolean inFuture() {
		normalize();
		return laterThan(now());
	}
	
	public boolean hasElapsed(Duration d) {
		return now().subtract(this).isLonger(d);
	}

	public String toString() {
		return secs + ":" + nsecs;
	}
}
