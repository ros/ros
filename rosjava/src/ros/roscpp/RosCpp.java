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

import java.util.Map;
import java.util.concurrent.ExecutionException;

import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;
import ros.ServiceClient;
import ros.ServiceServer;
import ros.Subscriber;
import ros.Topic;
import ros.communication.Time;

public class RosCpp extends Ros {
    private static class SingletonHolder { 
        private static final RosCpp instance = new RosCpp();
    }
    
    private boolean isInitialized;
    
    private RosCpp() {}
    
	public static RosCpp getInstance() {return SingletonHolder.instance; }

    
	@Override
	public void init(String name, boolean noSigintHandler, boolean anonymousName, boolean noRosout, String [] args) {
		if (isInitialized) throw new IllegalArgumentException("ROS has already been initialized.");
		if (args == null) args = new String[0];
		JNI.init(name, noSigintHandler, anonymousName, noRosout, args);
		isInitialized = true;
	}
    
	@Override
	public NodeHandle createNodeHandle(String ns, Map<String, String> remappings) {
		if (!isInitialized) throw new IllegalArgumentException("ROS has not been initialized.");
		return new CppNodeHandle(this, ns, remappings);
	}

	@Override
	public Time now() { return JNI.now(); }

	@Override
	public void spin() { JNI.spin(); }

	@Override
	public void spinOnce() { JNI.spinOnce(); }
	
	
	public void logDebug(String message) { JNI.logDebug(message); }
	public void logInfo(String message)  { JNI.logInfo(message); }
	public void logWarn(String message)  { JNI.logWarn(message); }
	public void logError(String message) { JNI.logError(message); }
	public void logFatal(String message) { JNI.logFatal(message); }

}
