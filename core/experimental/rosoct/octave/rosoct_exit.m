% varargout = rosoct_exit(varargin)
%
% 
function varargout = rosoct_exit(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('exit',varargin{:});
