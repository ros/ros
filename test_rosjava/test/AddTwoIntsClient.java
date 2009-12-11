import ros.*;
import ros.communication.*;
import ros.pkg.test_rosjava.srv.*;

public class AddTwoIntsClient {
    public static void main(String args[]) 
    throws InterruptedException, RosException, NumberFormatException {
        final Ros ros = Ros.getInstance();
		
		if (args.length != 2) {
			ros.logInfo("usage: add_two_ints_client X Y");
			System.exit(1); 
		}
		
        ros.init("AddTwoIntsClient");        
        NodeHandle n = ros.createNodeHandle();        
		
		ServiceClient<AddTwoInts.Request,AddTwoInts.Response,AddTwoInts>client= 
		n.serviceClient("add_two_ints" , new AddTwoInts(), false);
        
		AddTwoInts.Request rq = new AddTwoInts.Request();
		
		rq.a = Long.parseLong(args[0]); 
		rq.b = Long.parseLong(args[1]); 
		
		try {
			AddTwoInts.Response resp = client.call(rq); 
			ros.logInfo("Sum: " + resp.sum); 
		}
		catch (RosException e) {
			ros.logError("Failed to call service add_two_ints"); 
			System.exit(1); 
		}
		
		//If don't call n.shutdown, client thread will hang until ctrl+c
		n.shutdown(); 
    }
}