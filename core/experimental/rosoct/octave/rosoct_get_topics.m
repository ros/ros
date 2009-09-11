% varargout = rosoct_get_topics(varargin)
%
% 
function varargout = rosoct_get_topics(varargin)
varargout = cell(1,nargout);
[varargout{:}] = rosoct('get_topics',varargin{:});
