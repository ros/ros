#!/usr/bin/env python

PKG = 'rosfs'
import roslib; roslib.load_manifest(PKG)
import rospy

import os, stat, errno, sys
import fuse
import subprocess
from fuse import Fuse
from time import time
import types

fuse.fuse_python_api = (0, 2)

class MyStat(fuse.Stat):
    def __init__(self):
        self.st_mode = 0
        self.st_ino = 0
        self.st_dev = 0
        self.st_nlink = 0
        self.st_uid = 0
        self.st_gid = 0
        self.st_size = 0
        self.st_atime = 0
        self.st_mtime = 0
        self.st_ctime = 0


class ROSFS(Fuse):

    def __init__(self, version, usage, dash_s_do):
        Fuse.__init__(self, version=version, usage=usage, dash_s_do=dash_s_do)

        rospy.init_node("rosfs", anonymous=True)
        self._params = rospy.get_param("/")

        self.f = open("/tmp/rosfs.out", "w")
        
    def param_from_path(self, path, params):
        print "param_from_path", path, params
        if (path == "/"):
            return params

        keys = path.split('/')
        key = keys[1]
        if (not params.has_key(key)):
            print "  returning None for key", key
            return None

        val = params[key]
        if (len(keys) == 2):
            return val

        if (type(val) == types.DictType):
            return self.param_from_path('/' + '/'.join(keys[2:]), val)
        
        return None

    def is_top_level(self, path):
        return path in ['/', '/params']

    def getattr(self, path):
        print "getattr", path
        st = MyStat()
        st.st_atime = int(time())
        st.st_mtime = st.st_atime
        st.st_ctime = st.st_atime
        
        if (self.is_top_level(path)):
            st.st_mode = stat.S_IFDIR | 0755
            st.st_nlink = 1
        elif (path.startswith('/params/')):
            param = self.param_from_path(path[7:], self._params)
            if (param is None):
                print "  returning ENOENT(1)"
                return -errno.ENOENT
            
            if (type(param) == types.DictType):
                st.st_mode = stat.S_IFDIR | 0755
                st.st_nlink = len(param.keys())
            else:
                st.st_mode = stat.S_IFREG | 0755
                st.st_nlink = 1
                st.st_size = len(str(param) + "\n")
            
        return st

    def readdir(self, path, offset):
        print "readdir", path, offset
        yield fuse.Direntry('.')
        yield fuse.Direntry('..')

        dirs = {}
        if (path == "/"):
            print "  /"
            yield fuse.Direntry('params')
        elif (path == '/params'):
            dirs = self.param_from_path("/", self._params)
        elif (path.startswith('/params/')):
            dirs = self.param_from_path(path[7:], self._params)

        print type(dirs)
        for dir in dirs:
            print "   %s"%(dir)
            yield fuse.Direntry(dir)

    def open(self, path, flags):
        print "open", path, hex(flags)
       
        print flags, flags & 3, os.O_RDONLY
        if ((flags & 3) != os.O_RDONLY):
            print "  not readonly"
            return -errno.EACCES
        
        if (path.startswith('/params')):
            param = self.param_from_path(path[7:], self._params)
            if (param is None):
                print "  invalid param"
                return -errno.EACCES
        
            return str(param) + "\n"

        return -errno.EINVAL

    def read(self, path, size, offset, fh):
        print "read", path, size, offset, fh
        return fh[offset:size]

            
def main():
    usage="""
Userspace script runner
""" + Fuse.fusage
    server = ROSFS(version="%prog " + fuse.__version__,
                     usage=usage,
                     dash_s_do='setsingle')

    print >> server.f, "main"

    server.parse(errex=1)
    server.main()

if __name__ == '__main__':
    main()
