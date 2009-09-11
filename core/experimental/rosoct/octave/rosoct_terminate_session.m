% varargout = rosoct_terminate_session(varargin)
%
% 
function varargout = rosoct_terminate_session(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('terminate_session',varargin{:});
