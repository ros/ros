%% success = subscribe_simple(timelimit)
function success = subscribe_simple(timelimit)
startup;
success = rosoct_subscribe('chatter', @rosoct_String, @stringcb, 1);
if( ~success )
    error('failed to subscribe to chatter');
end

if( ~exist('timelimit','var') )
    timelimit = Inf;
end

tic;
while(toc < timelimit)
    rosoct_worker(20); % process 20 at a time
    if( ~rosoct_check_master() )
        error('master is down');
    end
end

success = rosoct_unsubscribe('chatter');
if( ~success )
    error('failed to unsubscribe from chatter');
end
