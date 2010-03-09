from __future__ import with_statement

class NamespaceConfig(object):
    """
    NamespaceConfig is a configuration object tied to a specific ROS
    namespace, like a Topics, Services, or Parameters.  It is used by
    a L{Namespace} to determine the namespace behavior.
    """
    
    def __init__(self, master, lock, list_fn, call_fn, slice_fn):
        self.cache = {}
        self.master = master
        self.list = list_fn
        self.lock = lock
        self.call = call_fn
        self.slice = slice_fn

class Namespace(object):
    """
    Namespace provides lightweight accessors to a ROS namespace and its data. The behavior of the namespace
    object is determined by a L{NamespaceConfig} instance.
    """
    
    def __init__(self, ns, config):
        self._ns = ns
        self._config = config
        self._type = None
        
    def trait_names(self):
        return list(set([s[len(self._ns):].split('/')[1] for s in self._config.list(self._ns)]))
    
    def __getattr__(self, key):
        if key in self.__dict__ or key.startswith('_'):
            return object.__getattr__(self, key)
        else:
            return self.__getitem__(key)

    def __iter__(self):
        return self._config.list(self._ns).__iter__()

    def __call__(self, *args, **kwds):
        self.config.call(*args, **kwds)

    def __getitem__(self, key):
        """
        Dictionary-style accessor for services
        """
        if type(key) == slice:
            self.config.slice_fn(key)
        else:
            key = roslib.names.ns_join(self._ns, key)
            config = self._config
            cache = config.cache

            if key in cache:
                return cache[key]
            else:
                with config.lock:
                    if key in cache:
                        obj = cache[key]
                    else:
                        obj = Namespace(key, config)
                        cache[key] = obj
                return obj
    
