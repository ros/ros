%% rosoct_test_function(testfn)
%%
%% Calls testfn and exits, return code is
%% 0 if function succeeds
%% 1 if function returns a success of 0
%% 2 if function throws an error
%%
function rosoct_test_function(testfn)

if( isoctave() )
    try
        success = testfn();
    catch
        display('problem occured');
        exit(2);
    end_try_catch
else % matlab
    try
        success = testfn();
    catch
        display('problem occured');
        exit(2);
    end
end

exit(success==0);
