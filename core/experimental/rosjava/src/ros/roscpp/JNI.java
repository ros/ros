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

import ros.ServiceServer;
import ros.Subscriber;
import ros.communication.Message;
import ros.communication.Time;

class JNI {
	private JNI() {}
	static { 
		System.loadLibrary("rosjava");
	}

	/************************************************************
	 *   Startup,  Shutdown, Running, and Node info
	 ************************************************************/

	public static native void init(String name, boolean noSigintHandler, boolean anonymousName, boolean noRosout, String[] args);

	public static native long createNodeHandle(String ns, Object[] remappings);
	public static native void shutdown(long cppHandle);
	
	public static native void spin();
	public static native void spinOnce();

	public static native boolean checkMaster(long cppHandle);
	public static native String getMasterHost(long cppHandle);
	public static native int getMasterPort(long cppHandle);
	public static native void setMasterRetryTimeout(long cppHandle, int ms);

	public static native String getName(long cppHandle);
	public static native String mapName(long cppHandle, String name);

	
	/************************************************************
	 *   Logging
	 ************************************************************/

	public static native void logDebug(String message);
	public static native void logInfo(String message);
	public static native void logWarn(String message);
	public static native void logError(String message);
	public static native void logFatal(String message);


	/************************************************************
	 *   Miscellaneous getters
	 ************************************************************/

	public static native Time now();

	public static native String [] getSubscribedTopics(long cppHandle);
	public static native String [] getAdvertisedTopics(long cppHandle);
	public static native String [] getPublishedTopics(long cppHandle);
	
	/************************************************************
	 *   Parameter server calls
	 ************************************************************/

	public static native boolean hasParam(long cppHandle, String param);

	public static native boolean getBooleanParam(long cppHandle, String param, boolean useCache);
	public static native int     getIntParam(long cppHandle, String param,    boolean useCache);
	public static native double  getDoubleParam(long cppHandle, String param, boolean useCache);
	public static native String  getStringParam(long cppHandle, String param, boolean useCache);
	
	public static native boolean setParam(long cppHandle, String param, boolean value);
	public static native boolean setParam(long cppHandle, String param, int     value);
	public static native boolean setParam(long cppHandle, String param, double  value);
	public static native boolean setParam(long cppHandle, String param, String  value);

	/************************************************************
	 *   Subscriptions and Publications
	 ************************************************************/

	@SuppressWarnings("unchecked")
	public static native long createSubCallback(Subscriber.Callback cb, Message template);
	public static native void deleteSubCallback(long cppCallback);
	public static native long subscribe(long cppHandle, String topic, long cppCallback, int queueSize);
	public static native boolean isSubscriberValid(long cppSubscriber);
	public static native void shutdownSubscriber(long cppSubscriber);

	public static native long advertise(long cppHandle, String topic, Message msgTemplate, int queueSize, boolean latch);
	public static native int getNumSubscribers(long cppPublisher);
	public static native void publish(long cppPublisher, Message m);
	public static native boolean isPublisherValid(long cppPublisher);
	public static native void shutdownPublisher(long cppPublisher);

	/************************************************************
	 *   Service Clients and Servers
	 ************************************************************/
	
	public static native long serviceClient(long cppHandle, String name, String md5, boolean isPersistant, Object [] headerValues);
	public static native boolean callService(long cppServiceClient, Message request, Message response, String md5);
	public static native void shutdownServiceClient(long cppServiceClient);

	@SuppressWarnings("unchecked")
	public static native long createSrvCallback(ServiceServer.Callback cb, String serviceMD5, String serviceDataType, Message reqTemplate, Message resTemplate);
	public static native void deleteSrvCallback(long cppCallback);
	public static native long advertiseService(long cppHandle, String name, long cppCallback);
	public static native boolean isServiceServerValid(long cppServiceServer);
	public static native void shutdownServiceServer(long cppServiceServer);

}
