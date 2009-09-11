more off;

[status,rosoctpath] = system('rospack find rosoct');
rosoctpath = strtrim(rosoctpath);
addpath(fullfile(rosoctpath, 'octave'));

if( ~rosoct_add_msgs('rosoct') )
    error('failed to add rosoct messages');
end
if( ~rosoct_add_srvs('rosoct') )
    error('failed to add rosoct services');
end

success = rosoct('shutdown');
if( ~success )
    error('failed to start rosoct');
end
