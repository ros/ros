% varargout = rosoct_unadvertise(varargin)
%
% 
function varargout = rosoct_unadvertise(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('unadvertise',varargin{:});
