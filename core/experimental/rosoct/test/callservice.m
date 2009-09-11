%% success = callservice()
function success = callservice(numcalls)
startup;
success = 0;
if( ~exist('numcalls','var') )
    numcalls = Inf;
end

success = rosoct_wait_for_service('mytempserv');
if( ~success )
    error('could not find service mytempserv');
    return;
end

i = 1;
failedcalls = 0;
while(i < numcalls)
    req = rosoct_StringString();
    req.str = sprintf('testing %d', i);
    res = rosoct_service_call('mytempserv',req);

    if( isempty(res) )
        if( isinf(numcalls) )
            failedcalls = failedcalls + 1;
            if( failedcalls > 10 )
                error('failed to call service (inf)!');
            end
            pause(0.2); % make calls at 20Hz
            continue;
        else
            error('failed to call service!');
        end
    end
    
    pause(0.05); % make calls at 20Hz
    display(res.str);
    i = i + 1;
    failedcalls = 0;
end
success = 1;
