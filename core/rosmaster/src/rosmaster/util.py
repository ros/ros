

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

