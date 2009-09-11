function ret = isoctave()
persistent isoct
if( isempty(isoct))
    isoct = exist('OCTAVE_VERSION') ~= 0;
end
ret = isoct;
