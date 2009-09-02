%% success = callsession(N)
function success = callsession(N)

%% first start roscpp_sessions/test/session_adv (the session server)
startup;
success = rosoct_add_srvs('roscpp_sessions');
if( ~success )
    return;
end

success = rosoct_wait_for_service('session_adv');
if( ~success )
    error('could not find service session_adv');
    return;
end

req = roscpp_sessions_simple_session();
req.sessionid = 0; % notifies server to create
[sessionid, res] = rosoct_create_session('session_adv', req);

if( isempty(sessionid) )
    error('failed to create session');
end

%% do small number of tests
tic;
if( ~exist('N','var') )
    N = 100;
end

for i = 1:N

    aa = floor(rand(1)*1000);
    bb = floor(rand(1)*1000);
    cc = aa+bb;

    reqset = roscpp_sessions_set_variable();
    reqget = roscpp_sessions_get_variable();
    reqadd = roscpp_sessions_add_variables();

    reqset.variable = 'aa';
    reqset.value = aa;
    resset = rosoct_session_call(sessionid,'set_variable',reqset);
    if( isempty(resset) )
        error('failed to call set_variable');
    end

    reqset.variable = 'bb';
    reqset.value = bb;
    resset = rosoct_session_call(sessionid,'set_variable',reqset);
    if( isempty(resset) )
        error('failed to call set_variable');
    end

    reqget.variable = 'aa';
    resget = rosoct_session_call(sessionid,'get_variable',reqget);
    if( isempty(resget) )
        error('failed to call get_variable');
    end

    if( resget.result ~= aa )
        resget.result
        aa
        error('resget bad value!');
    end

    reqadd.variable1 = 'aa';
    reqadd.variable2 = 'bb';
    reqadd.result = 'cc';
    resadd = rosoct_session_call(sessionid,'add_variables',reqadd);
    if( isempty(resget) )
        error('failed to call add_variables');
    end

    reqget.variable = 'cc';
    resget = rosoct_session_call(sessionid,'get_variable',reqget);
    if( isempty(resget) )
        error('failed to call get_variable');
    end

    if( resget.result ~= cc )
        error('resget bad add value!');
    end
end

disp(sprintf('complted %d octave session calls in %fs!',5*N,toc));
success = 1;
