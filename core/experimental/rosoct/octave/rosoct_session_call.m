% res = rosoct_session_call(sessionid, servicename, req)
%
% res - the response message, if empty, failed service call failed
function res = rosoct_session_call(sessionid, servicename, req)
res = req.create_response_();
[success, resdata] = rosoct('session_call', sessionid, servicename,req.server_md5sum_(),req.type_(),res.type_(),req.serialize_(req,0));
if( success )
    res = res.deserialize_(resdata);
else
    res = [];
end
