%% success = rosoct_add_msgs(pkg_name)
%%
%% adds the msgs in package to the path
function success = rosoct_add_msgs(pkg_name)

success = 0;
pkgpath = rosoct_findpackage(pkg_name);
if( ~isempty(pkgpath) )
    addpath(fullfile(pkgpath, 'msg','oct',pkg_name));
    success = 1;
end
