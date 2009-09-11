%% success = subscribe(numtimes)
function success = subscribe(numtimes)
startup;

[topics,success] = rosoct_get_topics('published')
if( ~success )
    error('failed to get topics');
end

if( ~exist('numtimes','var') )
    numtimes = Inf;
end

iter = 1;
while(iter < numtimes)
    success = rosoct_subscribe('chatter', @rosoct_String, @stringcb, 1);
    if( ~success )
        error('failed to subscribe to chatter');
    end

    tic;
    while(toc < 1)
        rosoct_worker(20);
    end

    success = rosoct_unsubscribe('chatter');
    if( ~success )
        error('failed to unsubscribe from chatter');
    end

    if( iter > 3 )
        rosoct('shutdown');
    end

    iter = iter+1;
end

display('success');
success = 1;