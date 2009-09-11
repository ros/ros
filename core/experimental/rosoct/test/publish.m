%% success = publish(numcalls)
function success = publish(numcalls)
startup;

%% test publishing a chatter topic
success = rosoct_advertise('chatter',@rosoct_String, 1);
if( ~success )
    error('failed to advertise chatter');
end

if( ~exist('numcalls','var') )
    numcalls = Inf;
end

i = 1;
while(i < numcalls)
    msg = rosoct_String();
    msg.data = sprintf('yo world %d', i);
    success = rosoct_publish('chatter',msg);
    if( ~success )
        error('failed to publish chatter');
    end
    sleep(0.01);
    i = i+1;
end

success = rosoct_unadvertise('chatter');
if( ~success )
    error('failed to unadvertise');
end
