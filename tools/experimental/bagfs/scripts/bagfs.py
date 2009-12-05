#!/usr/bin/env python

PKG = 'bagfs'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord

import os, stat, errno, sys
import fuse
import subprocess
from fuse import Fuse
from time import time

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


class BagFS(Fuse):

    def __init__(self, bag, version, usage, dash_s_do):
        Fuse.__init__(self, version=version, usage=usage, dash_s_do=dash_s_do)
        self.msgs = {}
        self.sizecache = {}
        for i,(topic, msg, t) in enumerate(rosrecord.logplayer(bag, raw=True)):
            cdir = self.msgs
            for ns in topic.lstrip('/').split('/'):
                if not ns in cdir:
                    cdir[ns] = {}
                cdir=cdir[ns]
            cdir["%.6d"%i] = (topic,msg,t)

    def lookup(self, path):
        cdir = self.msgs
        if path == '/':
            return cdir
        p = path.split('/')
        p.remove('')
        for ns in p[:-1]:
            if ns in cdir and isinstance(cdir[ns], dict):
                cdir=cdir[ns]
            else:
                return None
        if p[-1] in cdir:
            return cdir[p[-1]]
        else:
            return None

    def getattr(self, path):
        st = MyStat()
        st.st_atime = int(time())
        st.st_mtime = st.st_atime
        st.st_ctime = st.st_atime

        fil = self.lookup(path)
        if fil is None:
            return -errno.ENOENT                
        elif isinstance(fil, dict):
            st.st_mode = stat.S_IFDIR | 0755
            st.st_nlink = len(fil)
        else:
            st.st_mode = stat.S_IFREG | 0444
            st.st_nlink = 1
            st.st_mtime = int(fil[2].to_sec())
            st.st_ctime = st.st_mtime

            if path in self.sizecache:
                st.st_size = self.sizecache[path]
            else:
                st.st_size = 1
                
        return st            

    def readdir(self, path, offset):
        yield fuse.Direntry('.')
        yield fuse.Direntry('..')

        dir = self.lookup(path)

        for leaf in dir:
            yield fuse.Direntry(leaf)

    def open(self, path, flags):
        msg = self.lookup(path)
        accmode = os.O_RDONLY
        if (flags & accmode) != os.O_RDONLY:
            return -errno.EACCES

        m = msg[1][4]()
        m.deserialize(msg[1][1])
        
        disp = roslib.message.strify_message(m) + '\n'
        self.sizecache[path] = len(disp)

        return disp

    def read(self, path, size, offset, fh):
        return fh[offset:offset+size]

            
def main():
    usage="""
Userspace script runner
""" + Fuse.fusage
    bag = sys.argv[1]
    server = BagFS(bag, version="%prog " + fuse.__version__,
                     usage=usage,
                     dash_s_do='setsingle')

    server.parse(errex=1)
    server.main()

if __name__ == '__main__':
    main()
