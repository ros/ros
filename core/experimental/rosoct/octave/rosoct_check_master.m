% varargout = rosoct_check_master(varargin)
%
% 
function varargout = rosoct_check_master(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('check_master',varargin{:});
