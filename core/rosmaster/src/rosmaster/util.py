def canonicalize_name(name):
    """
    Put name in canonical form. Double slashes '//' are removed and
    name is returned without any trailing slash, e.g. /foo/bar
    @param name: ROS name
    @type  name: str
    """
    if not name or name == SEP:
        return name
    elif name[0] == SEP:
        return '/' + '/'.join([x for x in name.split(SEP) if x])
    else:
        return '/'.join([x for x in name.split(SEP) if x])        
    ##if len(name) > 1 and name[-1] == SEP:
    ##    return name[:-1]
    ##return name


_proxies = {} #cache ServerProxys
def xmlrpcapi(uri):
    """
    @return: instance for calling remote server or None if not a valid URI
    @rtype: xmlrpclib.ServerProxy
    """
    if uri is None:
        return None
    uriValidate = urlparse.urlparse(uri)
    if not uriValidate[0] or not uriValidate[1]:
        return None
    if not _proxies.has_key(uri):
        _proxies[uri] = xmlrpclib.ServerProxy(uri)
    return _proxies[uri]

