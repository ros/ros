% [sessionid, res] = rosoct_create_session(sessionname,req)
%
function [sessionid, res] = rosoct_create_session(sessionname, req)

res = req.create_response_();

if( ~isfield(req,'sessionid') || ~isfield(res,'sessionid') )
    error(['session ' sessionname ' request and response messages do not contain a sessionid field']);
end

sessionoffset = 0; % need a good way of determining this!

% verify
resnames = fieldnames(res);
if( !strcmp(resnames{1},'sessionid') )
    error(['sessionid in session type ' res.type_() ' not the first field!']);
end

[sessionid, resdata] = rosoct('create_session',sessionname,req.server_md5sum_(),req.type_(),res.type_(),@(seqid) req.serialize_(req,seqid), req.serializationLength_(req), sessionoffset);

if( ~isempty(sessionid) )
    res = res.deserialize_(resdata);
else
    res = [];
end
