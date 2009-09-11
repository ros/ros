% success = rosoct_advertise_service(servicename,msgfn,callback)
%
function varargout = rosoct_advertise_service(servicename,msgfn,callback)
[req,res] = msgfn();
varargout = cell(1,nargout);
[varargout{:}] = rosoct('advertise_service', servicename,req.server_md5sum_(), req.type_(), res.type_(), @(reqdata) rosoct_service_wrapper(reqdata,msgfn, callback));

function [success,resdata] = rosoct_service_wrapper(reqdata,msgfn,callback)
res = callback(msgfn().deserialize_(reqdata));
if( isempty(res) )
    success = 0;
    resdata = [];
else
    success = 1;
    resdata = res.serialize_(res,0);
end
