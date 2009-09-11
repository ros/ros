% success = rosoct_subscribe(topic,msgfn,callback,queuesize)
%
function varargout = rosoct_subscribe(topicname,msgfn,callback,queuesize)
msg = msgfn();
varargout = cell(1,nargout);
[varargout{:}] = rosoct('msg_subscribe',topicname,msg.md5sum_(),msg.type_(),@(data) callback(msg.deserialize_(data)),queuesize);
