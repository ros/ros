%% pkgpath = rosoct_findpackage(pkg_name)
%%
%% return the absolute path to the package
function pkgpath = rosoct_findpackage(pkg_name)
[status,pkgpath] = system(['rospack find ' pkg_name]);
if( status == 0 )
    pkgpath = strtrim(pkgpath);
else
    pkgpath = [];
end
