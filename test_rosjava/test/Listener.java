import ros.*;
import ros.communication.*;

public class Listener {
    public static void main(String args[]) 
    throws InterruptedException, RosException {
        final Ros ros = Ros.getInstance();
        ros.init("Listner");
        
        ros.pkg.std_msgs.msg.String msg = new ros.pkg.std_msgs.msg.String();
        
        NodeHandle n = ros.createNodeHandle();        
        
        Subscriber.Callback<ros.pkg.std_msgs.msg.String> callback = 
        new Subscriber.Callback<ros.pkg.std_msgs.msg.String>() {
            public void call(ros.pkg.std_msgs.msg.String msg) {
                ros.logInfo("Received [" + msg.data + "]"); 
            }
        };    
        
        Subscriber<ros.pkg.std_msgs.msg.String> sub; 
        sub = n.subscribe("chatter", 
                          new ros.pkg.std_msgs.msg.String(), 
                          callback, 
                          100);
        
        n.spin();
    }
}