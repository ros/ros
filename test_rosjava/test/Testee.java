
import ros.*;
import ros.communication.*;
import ros.pkg.test_rosjava.msg.*;
import ros.pkg.test_rosjava.srv.*;
import ros.pkg.std_msgs.msg.ByteMultiArray;
import ros.pkg.std_msgs.msg.MultiArrayLayout;
import ros.pkg.std_msgs.msg.MultiArrayDimension;

class Testee {
	public static void main(String [] args) throws InterruptedException, RosException{
		System.out.println("Starting rosjava.");
		Ros ros = Ros.getInstance();
		ros.init("testNode");
		
		// TODO: test these somehow?
		ros.logDebug("DEBUG");
		ros.logInfo("INFO");
		ros.logWarn("WARN");
		ros.logError("ERROR");
		ros.logFatal("FATAL");
		System.out.println(ros.now());
		
		System.out.println("Initialized");		
		NodeHandle n = ros.createNodeHandle();
		
		ros.pkg.std_msgs.msg.String msg = new ros.pkg.std_msgs.msg.String();
		msg.data = "go";
		Publisher<ros.pkg.std_msgs.msg.String> pub = n.advertise("/talk", msg, 1);
		
		Publisher<TestDataTypes> pub2 = n.advertise("/test-talk", new TestDataTypes(), 1);
		
		Publisher<TestBadDataTypes> pub3 = n.advertise("/test-bad", new TestBadDataTypes(), 1);
		
		
		Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String> cb = new Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String>(); 
		Subscriber<ros.pkg.std_msgs.msg.String> sub = n.subscribe("/listen", new ros.pkg.std_msgs.msg.String(), cb, 10);
		
		Subscriber.QueueingCallback<TestDataTypes> cb2 = new Subscriber.QueueingCallback<TestDataTypes>(); 
		Subscriber<TestDataTypes> sub2 = n.subscribe("/test-listen", new TestDataTypes(), cb2, 10);
		
		Subscriber.QueueingCallback<TestBadDataTypes> cb3 = new Subscriber.QueueingCallback<TestBadDataTypes>(); 
		Subscriber<TestBadDataTypes> sub3 = n.subscribe("/test-bad", new TestBadDataTypes(), cb3, 10);
		
		System.out.println("Waiting for roscpp...");
		for(int i = 0; i < 60 && cb.size() == 0; i++) {
			pub.publish(msg);
			n.spinOnce();
			Thread.sleep(1000);
		}
		System.out.println("Started");
		
		/// Test parameters
		msg.data = "good";
		if (n.getIntParam("int_param") != 1) msg.data="fail_int_param";
		n.setParam("int_param2", n.getIntParam("int_param"));
		if (n.getDoubleParam("double_param") != 1.0) msg.data="fail_double_param";
		n.setParam("double_param2", n.getDoubleParam("double_param"));
		if (n.getStringParam("string_param") == "hello") msg.data = "fail_string_param";
		n.setParam("string_param2", n.getStringParam("string_param"));
		

		// Test services
		ServiceServer.Callback<TestTwoInts.Request,TestTwoInts.Response> scb = 
			new ServiceServer.Callback<TestTwoInts.Request,TestTwoInts.Response>() {
				public TestTwoInts.Response call(TestTwoInts.Request request) {
					TestTwoInts.Response res = new TestTwoInts.Response();
					res.sum = request.a + request.b;
					return res;
				}
			};			
			
		ServiceServer<TestTwoInts.Request,TestTwoInts.Response,TestTwoInts> srv = n.advertiseService("add_two_ints_java", new TestTwoInts(), scb);
		
		pub.publish(msg);

		ServiceClient<TestTwoInts.Request, TestTwoInts.Response, TestTwoInts> sc = n.serviceClient("add_two_ints_cpp" , new TestTwoInts(), false);
		TestTwoInts.Request rq = new TestTwoInts.Request();
		rq.a = 12;
		rq.b = 17;
		msg.data="good";
		if (sc.call(rq).sum != 29) {
			System.out.println("Got incorrect sum!");
			msg.data="bad";
		}
		pub.publish(msg);
		
		// Test messages
		while(cb2.size() == 0) {
			n.spinOnce();
			Thread.sleep(1000);
		}

		msg.data="good";
		TestDataTypes test = cb2.pop();
		if ((0xff&(int)test.byte_) != 0xab)   msg.data="fail_byte";
		if ((0xff&(int)test.char_) != 0xbc)   msg.data="fail_char";
		if ((0xff&(int)test.uint8_) != 0xcd)  msg.data="fail_ui8";
		if ((0xff&(int)test.int8_) != 0xde)    msg.data="fail_i8";
		if ((0xffff&(int)test.uint16_) != 0xabcd) msg.data="fail_ui16";
		if ((0xffff&(int)test.int16_) != 0xbcde)   msg.data="fail_i16:" + test.int16_;
		if (test.uint32_ != 0xdeadbeef) msg.data="fail_ui32";
		if (test.int32_ != 0xcabcabbe)   msg.data="fail_i32";
		if (test.uint64_ != 0xbeefcabedeaddeedL) msg.data="fail_ui64";
		if (test.int64_ != 0xfeedbabedeadbeefL)   msg.data="fail_i64";
		if (test.float32_ != 1.0)   msg.data="fail_f32";
		if (test.float64_ != -1.0)  msg.data="fail_f64";
		if (!test.string_.equals("hello")) msg.data="fail_string";
		if (test.time_.secs != 123) msg.data="fail_time";
		if (test.time_.nsecs != 456) msg.data="fail_time";
		if (test.duration_.secs != 789) msg.data="fail_duration";
		if (test.duration_.nsecs != 987) msg.data="fail_duration";
		
		if (test.byte_v.length != 1) msg.data="fail_byte_v_len";
		if (test.byte_v[0] != 11) msg.data="fail_byte_v[0]";
		if (test.byte_f[0] != 22) msg.data="fail_byte_f[0]";
		if (test.byte_f[1] != 33) msg.data="fail_byte_f[1]";
		
		if (test.float64_v.length != 1) msg.data="fail_float64_v_len";
		if (test.float64_v[0] != 1.0) msg.data="fail_float64_v[0]";
		if (test.float64_f[0] != 2.0) msg.data="fail_float64_f[0]";
		if (test.float64_f[1] != 3.0) msg.data="fail_float64_f[1]";
		
		if (test.string_v.length != 1) msg.data="fail_string_v_len";
		if (!test.string_v[0].equals("test1")) msg.data="fail_string_v[0]";
		if (!test.string_f[0].equals("")) msg.data="fail_string_f[0]";
		if (!test.string_f[1].equals("test3")) msg.data="fail_string_f[1]";
		
		if (test.time_v.length != 1) msg.data="fail_time_v_len";
		if (test.time_v[0].secs != 222) msg.data="fail_time_v[0]";
		if (test.time_f[0].secs != 444) msg.data="fail_time_f[0]";
		if (test.time_f[1].secs != 666) msg.data="fail_time_f[1]";
		if (test.time_v[0].nsecs != 333) msg.data="fail_time_v[0]";
		if (test.time_f[0].nsecs != 555) msg.data="fail_time_f[0]";
		if (test.time_f[1].nsecs != 777) msg.data="fail_time_f[1]";
		
		if (test.Byte_.data != 1) msg.data="fail_Byte_";
		if (test.Byte_v.length != 2) msg.data="fail_Byte_v_length";
		if (test.Byte_v[0].data != (byte)2) msg.data="fail_Byte_v[0]";
		if (test.Byte_v[1].data != (byte)3) msg.data="fail_Byte_v[1]";

		if (test.ByteMultiArray_.layout.dim.length != 1) msg.data="fail_ByteMultiArray_layout_dims";
		if (!test.ByteMultiArray_.layout.dim[0].label.equals("test")) msg.data="fail_ByteMultiArray_layout_dim[0]_label";
		if (test.ByteMultiArray_.layout.dim[0].size != 1) msg.data="fail_ByteMultiArray_layout_dim[0]_size";
		if (test.ByteMultiArray_.layout.dim[0].stride != 1) msg.data="fail_ByteMultiArray_layout_dim[0]_stride";
		if (test.ByteMultiArray_.layout.data_offset != 0) msg.data="fail_ByteMultiArray_layout_data_offset";
		if (test.ByteMultiArray_.data.length != 1) msg.data="fail_ByteMultiArray_data_length";
		if (test.ByteMultiArray_.data[0] != (byte)11) msg.data="fail_ByteMultiArray_data[0]";

		if (test.ByteMultiArray_v.length != 1) msg.data="fail_ByteMultiArray_v_length";
		if (test.ByteMultiArray_v[0].layout.dim.length != 1) msg.data="fail_ByteMultiArray_v[0]layout_dims";
		if (!test.ByteMultiArray_v[0].layout.dim[0].label.equals("test")) msg.data="fail_ByteMultiArray_v[0]layout_dim[0]_label:" + test.ByteMultiArray_v[0].layout.dim[0].label;
		if (test.ByteMultiArray_v[0].layout.dim[0].size != 1) msg.data="fail_ByteMultiArray_v[0]layout_dim[0]_size";
		if (test.ByteMultiArray_v[0].layout.dim[0].stride != 1) msg.data="fail_ByteMultiArray_v[0]layout_dim[0]_stride";
		if (test.ByteMultiArray_v[0].layout.data_offset != 0) msg.data="fail_ByteMultiArray_v[0]layout_data_offset";
		if (test.ByteMultiArray_v[0].data.length != 1) msg.data="fail_ByteMultiArray_v[0]data_length";
		if (test.ByteMultiArray_v[0].data[0] != (byte)11) msg.data="fail_ByteMultiArray_v[0]data[0]";

//		TestDataTypes tmp_ = new TestDataTypes();
//		tmp_.deserialize(test.serialize(0));
		
		System.out.println("Result of good msg test: " + msg.data);
		pub2.publish(test);
		
		// Now, test the types that are still not built correctly by roscpp
		TestBadDataTypes tbdt = new TestBadDataTypes();
		tbdt.Byte_f[0].data = (byte)0xfe;
		tbdt.Byte_f[1].data = (byte)0xcd;
		tbdt.ByteMultiArray_f[0].layout.dim = new MultiArrayDimension[1];
		tbdt.ByteMultiArray_f[0].layout.dim[0] = new MultiArrayDimension();
		tbdt.ByteMultiArray_f[0].layout.dim[0].label="test";
		tbdt.ByteMultiArray_f[0].layout.dim[0].size=2;
		tbdt.ByteMultiArray_f[0].layout.dim[0].stride=1;
		tbdt.ByteMultiArray_f[0].layout.data_offset=0;
		tbdt.ByteMultiArray_f[0].data = new byte[2];
		tbdt.ByteMultiArray_f[0].data[0] = (byte)0xab;
		tbdt.ByteMultiArray_f[0].data[1] = (byte)0xdc;

		// Ensure we serialize and deserialize, in case roscpp does something fancy under the hood (i.e., direct transit)
		TestBadDataTypes temp = new TestBadDataTypes();
		temp.deserialize(tbdt.serialize(0)); 
		pub3.publish(temp);

//		pub3.publish(tbdt);
		
		while(cb3.size() == 0) {
			n.spinOnce();
		}
		tbdt = cb3.pop();
		if (tbdt.Byte_f.length != 2) msg.data="fail_Byte_f_len";
		if (tbdt.Byte_f[0].data != (byte)0xfe) msg.data="fail_Byte_f[0]";
		if (tbdt.Byte_f[1].data != (byte)0xcd) msg.data="fail_Byte_f[1]";
		if (tbdt.ByteMultiArray_f.length != 1) msg.data="fail_ByteMultiArray_f_length";
		if (tbdt.ByteMultiArray_f[0].layout.dim.length != 1) msg.data="fail_ByteMultiArray_f_dims";
		if (!tbdt.ByteMultiArray_f[0].layout.dim[0].label.equals("test")) msg.data="fail_ByteMultiArray_f_dim[0]_label";
		if (tbdt.ByteMultiArray_f[0].layout.dim[0].size != 2) msg.data="fail_ByteMultiArray_f_dim[0]_size";
		if (tbdt.ByteMultiArray_f[0].layout.dim[0].stride != 1) msg.data="fail_ByteMultiArray_f_dim[0]_stride";
		if (tbdt.ByteMultiArray_f[0].layout.data_offset != 0) msg.data="fail_ByteMultiArray_f_data_offset";
		if (tbdt.ByteMultiArray_f[0].data.length != 2) msg.data="fail_ByteMultiArray_f_data_length";
		if (tbdt.ByteMultiArray_f[0].data[0] != (byte)0xab) msg.data="fail_ByteMultiArray_f_data[0]";
		if (tbdt.ByteMultiArray_f[0].data[1] != (byte)0xdc) msg.data="fail_ByteMultiArray_f_data[1]";
		
		System.out.println("Result of bad msg test: " + msg.data);
		

		pub.publish(msg);
		
	
		/*	


			

			Subscriber.QueueingCallback<ros.pkg.rosjava_test.msg.String> cb = new Subscriber.QueueingCallback<ros.pkg.rosjava_test.msg.String>(); 
			Subscriber<ros.pkg.rosjava_test.msg.String> sub = n.subscribe("/chatter", new ros.pkg.rosjava_test.msg.String(), cb, 10);
			Thread.sleep(100);
			
			System.out.print("Published topics: ");
			for (Topic top : n.getTopics()) {
				System.out.print(top.getName() + ":" + top.getDatatype() + ", ");
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
		*/		
	}
}