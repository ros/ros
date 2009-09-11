%% success = advservice(numtimes)
function success = advservice(numtimes)
startup;

success = 0;
[req,res] = rosoct_StringString();

if( ~exist('numtimes','var') )
    numtimes = Inf;
end

iter = 1;
while(iter < numtimes)
    suc = rosoct_advertise_service('mytempserv',@rosoct_StringString,@stringservcb);
    if( ~suc )
        error('failed to advertise service!');
    end

    tic;
    while(toc < 1)
        rosoct_worker(20); % process 20 calls at a time
    end

    if( ~rosoct_check_master() )
        error('master is down');
    end
        
    success = rosoct_unadvertise_service('mytempserv');
    if( ~success ) 
        error('failed to unadvertise service');
    end

    if( iter > 2 )
        rosoct('shutdown');
    end

    iter = iter+1;
end

display('success');
success = 1;
