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
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.FutureTask;

import ros.RosException;
import ros.ServiceClient;
import ros.communication.Message;
import ros.communication.Service;

public class CppServiceClient<Q extends Message, A extends Message, S extends Service<Q, A> >  implements ServiceClient<Q, A, S> {
	private CppNodeHandle handle;
	private String name;
	private S serviceTemplate;
	private long cppServiceClient;
	private ExecutorService threadPool;
	
	private CppServiceClient() { }
	
	protected static <Q extends Message, A extends Message, S extends Service<Q, A> > CppServiceClient<Q,A,S> 
    create(CppNodeHandle handle, long cppHandle, String name, S serviceTemplate, boolean isPersistant, Map<String, String> headerValues) {
		CppServiceClient<Q, A, S> that = new CppServiceClient<Q, A, S>();
		that.handle = handle;
		that.name = name;
		that.serviceTemplate  = serviceTemplate;
		that.cppServiceClient = JNI.serviceClient(cppHandle, name, serviceTemplate.getMD5Sum(), isPersistant, Util.mapToArray(headerValues));
		if (that.cppServiceClient == 0)	throw new RuntimeException("Could not create service client " + name + "(should not happen)"); 
		return that;
	}

	public String getService() {return name; }
	
	public A call(Q request) throws RosException {
		A response = serviceTemplate.createResponse();
		if (!JNI.callService(cppServiceClient, request, response, serviceTemplate.getMD5Sum())) throw new RosException("Service call failed!");
		return response;
	}

	public FutureTask<A> callAsync(final Q request) throws RosException {
		if (threadPool == null) threadPool = Executors.newCachedThreadPool();
		final CppServiceClient<Q, A, S> that = this;
		FutureTask<A> f = new FutureTask<A>(new Callable<A>() {
			public A call() throws Exception {
				return (A) that.call(request);
			}
  		});
		handle.submitFuture(f);
		return f;
	}

	public A callLocal(Q request) throws RosException {
		FutureTask<A> f = callAsync(request);
		while(!f.isDone()) {
			handle.spinOnce();
		}
		try {
			return f.get();
		} catch (InterruptedException e) {
			throw new RosException("InterruptedException in callLocal", e);
		} catch (ExecutionException e) {
			throw new RosException("ExecutionException in callLocal", e);
		}
	}

	public boolean isValid() { return (cppServiceClient != 0); }

	public void shutdown() {
		if (!isValid()) return;
		JNI.shutdownServiceClient(cppServiceClient);
		cppServiceClient = 0;
	}
}

