% success = rosoct_advertise(topic, msgfn, queuesize)
function varargout = rosoct_advertise(topic, msgfn, queuesize)
msg = msgfn();
varargout = cell(1,nargout);
[varargout{:}] = rosoct('advertise', topic, msg.md5sum_(), msg.type_(), queuesize);
