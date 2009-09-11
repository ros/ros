% varargout = rosoct_worker(varargin)
%
% 
function varargout = rosoct_worker(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('worker',varargin{:});
