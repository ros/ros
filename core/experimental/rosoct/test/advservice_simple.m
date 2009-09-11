%% success = advservice_simple()
function success = advservice_simple(timelimit)
startup;
rosoct_unadvertise_service('mytempserv');
success = rosoct_advertise_service('mytempserv',@rosoct_StringString,@stringservcb);
if( ~success )
    error('failed to advertise service!');
end

if( ~exist('timelimit','var') )
    timelimit = Inf;
end

tic;
while(toc < timelimit)
    rosoct_worker(20); % process 20 calls at a time
    if( ~rosoct_check_master() )
        error('master is down');
    end
end
