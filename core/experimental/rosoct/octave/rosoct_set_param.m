% varargout = rosoct_set_param(varargin)
%
% 
function varargout = rosoct_set_param(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('set_param',varargin{:});
