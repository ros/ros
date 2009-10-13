% success = rosoct_publish(topic, msg)
function varargout = rosoct_publish(topicname, msg)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('publish',topicname,msg.md5sum_(),@(seqid) msg.serialize_(msg,seqid),msg.serializationLength_(msg), msg.type_());
