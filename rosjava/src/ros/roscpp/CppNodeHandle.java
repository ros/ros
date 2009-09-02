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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.FutureTask;

import ros.NodeHandle;
import ros.Publisher;
import ros.RosException;
import ros.ServiceClient;
import ros.ServiceServer;
import ros.Subscriber;
import ros.Topic;
import ros.ServiceServer.Callback;
import ros.communication.Message;
import ros.communication.Service;
// TODO: see why we get messages out of order?
import ros.communication.Time;

public class CppNodeHandle extends NodeHandle {
	private RosCpp ros;
	private long cppHandle;
	private boolean isValid;
	String nsArg;
	private Map<String, String> remappingArgs;
	private List<CppPublisher<Message> > publishers;
	private List<CppSubscriber<Message> > subscribers;
	private List<CppServiceServer<Message,Message,Service<Message, Message> > > serviceServers;
	private List<CppServiceClient<Message,Message,Service<Message, Message> > > serviceClients;

	private ExecutorService threadPool;
	
	protected <V> void submitFuture(FutureTask<V> f) {
		if (threadPool == null) threadPool = Executors.newCachedThreadPool();
		threadPool.submit(f);
	}

	
	/************************************************************
	 *   Node Handle creation, destruction, and features
	 ************************************************************/
	
	protected CppNodeHandle(RosCpp ros, String ns, Map<String, String> remappings) {
		this.ros = ros;
		nsArg = ns;
		remappingArgs = remappings;
		publishers = new LinkedList<CppPublisher<Message> > ();
		subscribers = new LinkedList<CppSubscriber<Message> > ();
		serviceServers = new LinkedList<CppServiceServer<Message,Message,Service<Message, Message> > >();
		serviceClients = new LinkedList<CppServiceClient<Message,Message,Service<Message, Message> > >();
		cppHandle = JNI.createNodeHandle(ns, Util.mapToArray(remappings));
		if (cppHandle == 0) throw new RuntimeException("Could not create node handle (should never happen)."); 
		
		isValid = true;
	}

	protected void finalize() { shutdown(); }

	public NodeHandle copy() { return new CppNodeHandle(ros, nsArg, remappingArgs); }

	public void shutdown() {
		if (!isValid()) throw new RuntimeException("Node has already been shutdown");
		for (CppPublisher<Message> pub : publishers) pub.shutdown();
		for (CppSubscriber<Message> sub : subscribers) sub.shutdown();
		for (CppServiceServer<Message,Message,Service<Message, Message> > ss : serviceServers) ss.shutdown();
		for (CppServiceClient<Message,Message,Service<Message, Message> > sc : serviceClients) sc.shutdown();
		JNI.shutdown(cppHandle);
		cppHandle = 0;
		if (threadPool != null) threadPool.shutdownNow();
		isValid = false;
	}

	public boolean isValid()     { return isValid; } 
	
	
	public boolean checkMaster() { return JNI.checkMaster(cppHandle); }
	public String getMasterHost() { return JNI.getMasterHost(cppHandle); }
	public int getMasterPort() { return JNI.getMasterPort(cppHandle); }
	public void setMasterRetryTimeout(int ms) { JNI.setMasterRetryTimeout(cppHandle, ms); }

	public String getName()      { return JNI.getName(cppHandle); }
	public String getNamespace() { return nsArg;}
	public String resolveName(String name) { return JNI.mapName(cppHandle, name); }



	/************************************************************
	 *   Misc. getters
	 ************************************************************/

	public Collection<Topic> getTopics() {
		String [] topics = JNI.getPublishedTopics(cppHandle);
		int len = topics.length / 3;
		if (len * 3 != topics.length) throw new RuntimeException("Unexpected output from getPublishedTopics");
		List<Topic> ret = new ArrayList<Topic>();
		for(int i = 0; i < len; i++) {
			ret.add(new Topic(topics[i*3 + 0], topics[i*3 + 1], topics[i*3 + 2]));
		}
		return ret;
	}

	public Collection<String> getSubscribedTopics() {
		return (List<String>)Arrays.asList(JNI.getSubscribedTopics(cppHandle));
	}

	public Collection<String> getAdvertisedTopics() {
		return (List<String>)Arrays.asList(JNI.getAdvertisedTopics(cppHandle));
	}

	/************************************************************
	 *   Parameter server calls
	 ************************************************************/

	public boolean hasParam(String param) {
		return JNI.hasParam(cppHandle, param);
	}

	public boolean getBooleanParam(String param, boolean useCache) throws RosException {
		return JNI.getBooleanParam(cppHandle, param, useCache); 
	}
	public int     getIntParam(String param,    boolean useCache)  throws RosException {
		return JNI.getIntParam(cppHandle, param, useCache); 
	}
	public double  getDoubleParam(String param, boolean useCache)  throws RosException {
		return JNI.getDoubleParam(cppHandle, param, useCache); 
	}
	public String  getStringParam(String param, boolean useCache)  throws RosException {
		return JNI.getStringParam(cppHandle, param, useCache); 
	}
	
	public void setParam(String param, boolean value) {JNI.setParam(cppHandle, param, value);}
	public void setParam(String param, int     value) {JNI.setParam(cppHandle, param, value);}
	public void setParam(String param, double  value) {JNI.setParam(cppHandle, param, value);}
	public void setParam(String param, String  value) {
		if (param == null) throw new IllegalArgumentException("Can't set string parameter " + param + " to null");
		JNI.setParam(cppHandle, param, value);
	}

	
	/************************************************************
	 *   Publications and Subscriptions
	 ************************************************************/

	@SuppressWarnings("unchecked")
	public <M extends Message> Subscriber<M> subscribe(String topic, M messageTemplate, 
                                                 		ros.Subscriber.Callback<M> callback, int queueSize) throws RosException {
		Subscriber<M> ret = CppSubscriber.create(cppHandle, topic, messageTemplate, callback,queueSize);
		subscribers.add((CppSubscriber<Message>) ret);
		return ret;
	}

	@SuppressWarnings("unchecked")
	public <M extends Message> Publisher<M> advertise(String newTopic, M messageTemplate, int queueSize, boolean latch) throws RosException {
		Publisher<M> ret = CppPublisher.create(cppHandle, newTopic, messageTemplate, queueSize, latch);
		publishers.add((CppPublisher<Message>) ret);
		return ret;
	}


	/************************************************************
	 *   Service Clients and Servers
	 ************************************************************/

	@SuppressWarnings("unchecked")
	public <Q extends Message, A extends Message, S extends Service<Q, A>> ServiceClient<Q, A, S> serviceClient(
			String serviceName, S serviceTemplate, boolean isPersistant, Map<String, String> headerValues) {
		CppServiceClient<Q,A,S> ret = CppServiceClient.create(this, cppHandle, serviceName, serviceTemplate, isPersistant, headerValues);
		serviceClients.add((CppServiceClient)ret);
		return ret;
	}


	@SuppressWarnings("unchecked")
	public <Q extends Message, A extends Message, S extends Service<Q, A>> ServiceServer<Q, A, S> advertiseService(
			String serviceName, S serviceTemplate, Callback<Q, A> callback) throws RosException {
		CppServiceServer<Q,A,S> ret = CppServiceServer.create(cppHandle, serviceName, serviceTemplate, callback);
		serviceServers.add((CppServiceServer) ret);
		return ret;
	}


	
	/************************************************************
	 *   Convenience methods copied from Ros
	 ************************************************************/

	public Time now()                      { return ros.now(); }

	public void spin()                     { ros.spin(); }
	public void spinOnce()                 { ros.spinOnce(); }

	public void logDebug(String message)   { ros.logDebug(message); }
	public void logInfo(String message)    { ros.logInfo(message); }
	public void logWarn(String message)    { ros.logWarn(message); }
	public void logError(String message)   { ros.logError(message); }
	public void logFatal(String message)   { ros.logFatal(message); }
	
	
}
