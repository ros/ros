%% success = rosoct_add_srvs(pkg_name)
%%
%% adds the msgs in package to the path
function success = rosoct_add_srvs(pkg_name)

success = 0;
pkgpath = rosoct_findpackage(pkg_name);
if( ~isempty(pkgpath) )
    addpath(fullfile(pkgpath, 'srv','oct',pkg_name));
    success = 1;
end
