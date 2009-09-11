% varargout = rosoct_unsubscribe(varargin)
%
% 
function varargout = rosoct_unsubscribe(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('msg_unsubscribe',varargin{:});
