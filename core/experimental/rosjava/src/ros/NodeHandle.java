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

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import ros.communication.*;

public abstract class NodeHandle {
	
	// Node and master info
	public abstract NodeHandle copy();
	public abstract String getNamespace();
	public abstract String resolveName(String name);
	public abstract String getName();
//	public abstract boolean ok(); ??
	public abstract boolean isValid();
	public abstract void shutdown();

	public abstract boolean checkMaster();
	public abstract String getMasterHost();
	public abstract int getMasterPort();
	public abstract void setMasterRetryTimeout(int ms);

	
	// Getting and setting parameters
    // TODO: support for XmlRpc values
	
	public abstract boolean hasParam(String param);

	public abstract boolean getBooleanParam(String param, boolean useCache) throws RosException;
	public abstract int     getIntParam(String param, boolean useCache)     throws RosException;
	public abstract double  getDoubleParam(String param, boolean useCache)  throws RosException;
	public abstract String  getStringParam(String param, boolean useCache)  throws RosException;
	
	public boolean getBooleanParam(String param)  throws RosException {return getBooleanParam(param, false); }
	public int     getIntParam(String param)      throws RosException {return getIntParam(param, false); }
	public double  getDoubleParam(String param)   throws RosException {return getDoubleParam(param, false); }
	public String  getStringParam(String param)   throws RosException {return getStringParam(param, false); }

	public abstract void setParam(String param, boolean value) throws RosException;
	public abstract void setParam(String param, int     value);
	public abstract void setParam(String param, double  value);
	public abstract void setParam(String param, String  value);

	
	// Listing publications and subscriptions

	public abstract Collection<Topic> getTopics();
	public abstract Collection<String>  getAdvertisedTopics();
	public abstract Collection<String>  getSubscribedTopics();
	

	
	// Managing publications and subscriptions
	// TODO: support for custom callback queues or subscription callbacks
	
	public abstract <M extends Message> Publisher<M> advertise(String newTopic, M messageTemplate, int queueSize, boolean latch) throws RosException;
	
	public <M extends Message> Publisher<M> advertise(String newTopic, M messageTemplate, int queueSize) throws RosException {
		return advertise(newTopic, messageTemplate, queueSize, false);
	}

	public abstract <M extends Message> Subscriber<M> subscribe(String topic, M messageTemplate, Subscriber.Callback<M> callback, int queueSize)  throws RosException;

	public abstract <Q extends Message, A extends Message, S extends Service<Q, A> > ServiceServer<Q, A, S> 
	                advertiseService(String serviceName, S serviceTemplate, ServiceServer.Callback<Q,A> callback)  throws RosException;

	public abstract <Q extends Message, A extends Message, S extends Service<Q, A> > ServiceClient<Q, A, S> 
					serviceClient(String serviceName, S serviceTemplate, boolean isPersistant, Map<String, String> headerValues);

	public <Q extends Message, A extends Message, S extends Service<Q, A> > ServiceClient<Q, A, S> 
                    serviceClient(String serviceName, S serviceTemplate) {
		return serviceClient(serviceName, serviceTemplate, false);
	}

	public <Q extends Message, A extends Message, S extends Service<Q, A> > ServiceClient<Q, A, S> 
                    serviceClient(String serviceName, S serviceTemplate, boolean isPersistant) {
		return serviceClient(serviceName, serviceTemplate, isPersistant, new HashMap<String, String>());
	}

	// Convenience methods copied from Ros
	
	public abstract Time now();
	public abstract void spin();
	public abstract void spinOnce();
	public abstract void logDebug(String message);
	public abstract void logInfo(String message);
	public abstract void logWarn(String message);
	public abstract void logError(String message);
	public abstract void logFatal(String message);
}
