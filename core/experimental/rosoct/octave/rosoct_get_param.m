% varargout = rosoct_get_param(varargin)
%
% 
function varargout = rosoct_get_param(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('get_param',varargin{:});
