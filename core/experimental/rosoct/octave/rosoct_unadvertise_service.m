% varargout = rosoct_unadvertise_service(varargin)
%
% 
function varargout = rosoct_unadvertise_service(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('unadvertise_service',varargin{:});
