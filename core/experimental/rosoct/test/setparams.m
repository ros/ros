%% success = setparams()
function success = setparams()
startup;

success = rosoct_set_param('myparami',1);
if( ~success )
    return;
end
success = rosoct_set_param('myparamd',2.0);
if( ~success )
    return;
end
success = rosoct_set_param('myparams','nothing');
if( ~success )
    return;
end

i = rosoct_get_param('myparami');
if( isempty(i) )
    error('failed to get myparami');
end
d = rosoct_get_param('myparamd');
if( isempty(d) )
    error('failed to get myparamd');
end
s = rosoct_get_param('myparams');
if( isempty(s) )
    error('failed to get myparams');
end

if( i ~= 1 || d ~= 2.0 || ~strcmp(s,'nothing') )
    error('bad param setting!');
end
