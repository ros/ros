% res = rosoct_service_call(servicename,req)
%
% res - the response message, if empty, failed service call failed
function res = rosoct_service_call(servicename,req)
res = req.create_response_();
[success, resdata] = rosoct('service_call',servicename,req.server_md5sum_(),req.type_(),res.type_(),req.serialize_(req,0));
if( success )
    res = res.deserialize_(resdata);
else
    res = [];
end
