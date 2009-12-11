import ros.*;
import ros.communication.*;

public class Talker {
	public static void main(String args[]) 
	                       throws InterruptedException, RosException {
		Ros ros = Ros.getInstance();
		ros.init("Talker");
				
		ros.pkg.std_msgs.msg.String msg = new ros.pkg.std_msgs.msg.String();
		
		NodeHandle n = ros.createNodeHandle();		
		Publisher<ros.pkg.std_msgs.msg.String> pub = 
							               n.advertise("chatter", msg, 100);
		
		int count = 0; 
		while(pub.isValid()) {
			msg.data = "Hello there! This is message [" + count + "]"; 
			pub.publish(msg);
			ros.logInfo("I published [" + msg.data + "]"); 
			n.spinOnce();
			Thread.sleep(200);
			count++; 
		}
	}
}