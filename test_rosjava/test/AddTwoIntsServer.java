import ros.*;
import ros.communication.*;
import ros.pkg.test_rosjava.srv.*;

public class AddTwoIntsServer {
    public static void main(String args[]) 
    throws InterruptedException, RosException {
        final Ros ros = Ros.getInstance();
        ros.init("AddTwoIntsServer");
        
        NodeHandle n = ros.createNodeHandle();        
        
        ServiceServer.Callback<AddTwoInts.Request,AddTwoInts.Response> scb = 
        new ServiceServer.Callback<AddTwoInts.Request,AddTwoInts.Response>() {
            public AddTwoInts.Response call(AddTwoInts.Request request) {
                AddTwoInts.Response res = new AddTwoInts.Response();
                res.sum = request.a + request.b;
                ros.logInfo("request: x=" + request.a + ", y=" + request.b);
                ros.logInfo("sending back response: " + res.sum); 
                return res;
            }
        };            
        
        ServiceServer<AddTwoInts.Request,AddTwoInts.Response,AddTwoInts> srv = 
        n.advertiseService("add_two_ints", new AddTwoInts(), scb);
        
        ros.logInfo("Ready to add two ints."); 
        
        n.spin();
    }
}