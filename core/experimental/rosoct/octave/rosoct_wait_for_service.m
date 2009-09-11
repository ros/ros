% varargout = rosoct_wait_for_service(varargin)
%
% 
function varargout = rosoct_wait_for_service(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('wait_for_service',varargin{:});
