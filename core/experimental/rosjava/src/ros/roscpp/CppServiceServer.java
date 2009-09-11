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
import ros.ServiceServer;
import ros.communication.Message;
import ros.communication.Service;

public class CppServiceServer<Q extends Message, A extends Message, S extends Service<Q, A> >  implements ServiceServer<Q, A, S> {
	private String name;
	private long cppCallback;
	private long cppServiceServer;
	
	private CppServiceServer() { }
	
	protected static <Q extends Message, A extends Message, S extends Service<Q, A> > CppServiceServer<Q,A,S> 
    create(long cppHandle, String name, S serviceTemplate,  Callback<Q, A> callback) throws RosException {
		CppServiceServer<Q, A, S> that = new CppServiceServer<Q, A, S>();
		that.name = name;
		that.cppCallback = JNI.createSrvCallback(callback, serviceTemplate.getMD5Sum(), serviceTemplate.getDataType(), 
					                             serviceTemplate.createRequest(), serviceTemplate.createResponse());
		if (that.cppCallback == 0) throw new RuntimeException("Could not create callback wrapper.");
		that.cppServiceServer = JNI.advertiseService(cppHandle, name, that.cppCallback);
		if (that.cppServiceServer == 0)	{
			JNI.deleteSrvCallback(that.cppCallback);
			throw new RosException("Could not create service server " + name);
		}
		return that;
	}

	public String getService() {return name; }
	
	public boolean isValid() { return (cppServiceServer != 0) && JNI.isServiceServerValid(cppServiceServer); }

	public void shutdown() {
		if (!isValid()) return;
		JNI.shutdownServiceServer(cppServiceServer);
		JNI.deleteSrvCallback(cppCallback);
		cppServiceServer = 0;
	}
}
