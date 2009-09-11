% varargout = rosoct_get_param(varargin)
%
% 
function varargout = rosoct_time_now(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('time_now',varargin{:});
