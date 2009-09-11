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
import ros.communication.Time;
import ros.pkg.rosjava_test.srv.TwoInts;

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
	
	
	
	/************************************************************
	 *   Test
	 ************************************************************/

	
	public static void main(String [] args) throws InterruptedException, RosException, ExecutionException {
		System.out.println("Starting");
		Ros ros = Ros.getInstance();
		ros.logDebug("DEBUG");
		ros.logInfo("INFO");
		ros.logWarn("WARN");
		ros.logError("ERROR");
		ros.logFatal("FATAL");
		ros.init("testNode");
		NodeHandle n = ros.createNodeHandle();
		System.out.println("Initialized");
		n.setParam("test", 2);
		n.setParam("test2", 2.2);
		n.setParam("test3", "2.5");
		System.out.println("Set");
		System.out.println(n.getIntParam("test"));
		System.out.println(n.getDoubleParam("test2"));
		System.out.println(n.getStringParam("test3"));
		System.out.println(ros.now());
		
		
		ServiceClient<TwoInts.Request, TwoInts.Response, TwoInts> sc = n.serviceClient("add_two_ints" , new TwoInts(), false);
		TwoInts.Request rq = new TwoInts.Request();
		rq.a = 12;
		rq.b = 17;
		try {
			TwoInts.Response res = sc.call(rq);
			System.out.println("ERROR: got no exception:" + res.sum);
		} catch (RosException e){
			System.out.println("Got exception as expected");
		}
		
		ServiceServer.Callback<TwoInts.Request,TwoInts.Response> scb = 
			new ServiceServer.Callback<TwoInts.Request,TwoInts.Response>() {
				public TwoInts.Response call(TwoInts.Request request) {
					TwoInts.Response res = new TwoInts.Response();
					res.sum = request.a + request.b;
					return res;
				}
			};			
		ServiceServer<TwoInts.Request,TwoInts.Response,TwoInts> srv = n.advertiseService("add_two_ints", new TwoInts(), scb);
/*		Future<TwoInts.Response> f = sc.callAsync(rq);
		ros.spinOnce();
		System.out.println("12 + 17 = " + f.get().sum);
*/
		rq.a = -17;
		System.out.println("-17 + 17 = " + sc.callLocal(rq).sum);

		
		Publisher<ros.pkg.rosjava_test.msg.String> pub = n.advertise("/chatter", new ros.pkg.rosjava_test.msg.String(), 100);
		Subscriber.QueueingCallback<ros.pkg.rosjava_test.msg.String> cb = new Subscriber.QueueingCallback<ros.pkg.rosjava_test.msg.String>(); 
		Subscriber<ros.pkg.rosjava_test.msg.String> sub = n.subscribe("/chatter", new ros.pkg.rosjava_test.msg.String(), cb, 10);
		Thread.sleep(100);
		
		System.out.print("Published topics: ");
		for (Map.Entry<String, String> e : n.getPublishedTopics().entrySet()) {
			System.out.print(e.getKey() + ":" + e.getValue() + ", ");
		}
		System.out.println();

		System.out.print("Advertised topics: ");
		for (String s : n.getAdvertisedTopics()) {
			System.out.print(s  + ", ");
		}
		System.out.println();

		System.out.print("Subscribed topics: ");
		for (String s : n.getSubscribedTopics()) {
			System.out.print(s  + ", ");
		}
		System.out.println();

		
		for(int i = 0; i < 50; i++) {
			System.out.println(i);
			ros.pkg.rosjava_test.msg.String m = new ros.pkg.rosjava_test.msg.String();
			m.data = "Hola " + i;
			pub.publish(m);
			if (i == 37) sub.shutdown();

			while (!cb.isEmpty()) {
				System.out.println(cb.pop().data);
			}
			Thread.sleep(100);
			ros.spinOnce();
		}
		
		pub.shutdown();
		srv.shutdown();
		sc.shutdown();

		n.shutdown();
	} 
	
}
