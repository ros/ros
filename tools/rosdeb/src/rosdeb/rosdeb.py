#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
usage: %(progname)s [package|all]
  --nobuild - do not build tree

Builds a debian package of a ROS package.

apt-get install fakeroot
"""

import os, sys, string, time, getopt

import re
import subprocess
import stat

import roslib.manifest

#_DEBUG = False
_DEBUG = True

class DebianBuildError(Exception):
  pass

def makedir(path):
  if os.path.exists(path): return
  os.makedirs(path)

def toDebianName(pkg):
  if pkg == "ros-core": return pkg
  pkgname = pkg.lower()
  pkgname = pkgname.replace("_", "-")
  pkgname = "ros-" + pkgname 

  return pkgname

def getOSDict():
  d = {}
  for line in open("/etc/lsb-release"):
    line = line.strip()
    if not line: continue
    parts = line.split("=", 1)
    d[parts[0]] = parts[1]
  return d

def getOS():
  d = getOSDict()
  ostype = d['DISTRIB_ID'].lower()
  osversion = d['DISTRIB_RELEASE']

  return ostype, osversion

def getSVNVersionSVN(path):
  versionStr = subprocess.Popen(["svn", "info", path], stdout=subprocess.PIPE).communicate()[0].strip()
  pat = re.compile("Last Changed Rev: ([0-9]+)", re.I)
  m = pat.search(versionStr)
  if m:
    version = m.group(1)
  else:
    version = None
  return version

def getSVNVersionGIT(path):
  versionStr = subprocess.Popen(["git-log", "."], cwd=path, stdout=subprocess.PIPE).communicate()[0].strip()

  pat = re.compile("git-svn-id: .+@([0-9]+)", re.I)
  m = pat.search(versionStr)
  if m:
    version = m.group(1)
  else:
    version = "0"
  return version

def getSVNVersion(path):
  ros_root = os.environ["ROS_ROOT"]
  if os.path.isdir(os.path.join(ros_root, ".git")):
    return getSVNVersionGIT(path)
  else:
    return getSVNVersionSVN(path)

def getPackageDeps(pkg):
  depstr = subprocess.Popen(["rospack", "deps", pkg], stdout=subprocess.PIPE).communicate()[0]
  deps = []
  if depstr: deps = depstr.strip().split("\n")

  return deps

def getPackagePath(pkg):
  pkgpath = subprocess.Popen(["rospack", "find", pkg], stdout=subprocess.PIPE).communicate()[0].strip()
  return pkgpath

class DebianPackager:
  def __init__(self, osdict, arch=None):
    self.rosCorePackagesDict = self.rosCorePackages()
    self.buildFlag = True
    self.osdict = osdict
    self.arch = arch

    self.rospath = os.path.join("usr", "lib", "ros")

  def setupTemporaryDirectory(self):
    ## setup temporary directory
    self.tmpdir = os.path.join(os.sep, 'tmp', 'rostar-%s' % (os.getpid()))
    if os.path.exists(self.tmpdir): self.removeTmpDir()

    os.mkdir(self.tmpdir)

    self.debianpath = os.path.join(self.tmpdir, "DEBIAN")
    self.binpath = os.path.join(self.tmpdir, self.rospath, "bin")
    self.pkgspath = os.path.join(self.tmpdir, self.rospath, "pkgs")
    self.libpath = os.path.join(self.tmpdir, self.rospath, "lib")
    self.pythonpath = os.path.join(self.tmpdir, self.rospath, "python")
    self.usrbinpath = os.path.join(self.tmpdir, "usr", "bin")

    ## pre-create the directories

    makedir(self.pythonpath)
    makedir(self.libpath)
    makedir(self.usrbinpath)
    makedir(self.pkgspath)
    makedir(self.binpath)
    makedir(self.debianpath)

  def removeTmpDir(self):
    os.system('rm -rf %s' % (self.tmpdir))
    

  def buildRosCore(self):
    packages = {}

    corepackages = self._rosCorePackages()
    for pkg in corepackages:
      syspackages = self.getSysDepend(pkg)
      for _pkg in syspackages:
        packages[_pkg] = 1

    packages['ros-boost'] = 1
    packages['ros-log4cxx'] = 1

    req_packages = 'build-essential python-dev subversion pkg-config libapr1-dev libaprutil1-dev wget zip cmake python-yaml zlib1g-dev libbz2-dev'.split()
    for p in req_packages: packages[p] = 1

    packages = packages.keys()
    packages.sort()
    #print "packages", packages

    rosRoot = os.environ["ROS_ROOT"]
    for _path in ("bin", "config"):
      ipath = os.path.join(rosRoot, _path)
      jpath = os.path.join(self.tmpdir, self.rospath, _path)
      if not os.path.exists(jpath): makedir(jpath)
      files = os.listdir(ipath)
      for fn in files:
        fullfn = os.path.join(ipath, fn)

        if os.path.isdir(fullfn): continue
        if fn.find(".backup-") != -1: continue
        if fn.find(".svn") != -1: continue

        tmpfullfn = os.path.join(jpath, fn)
        cmd = "cp -r %s %s" % (fullfn, tmpfullfn)
        os.system(cmd)

      if _path == "bin":
        rosbusybox = os.path.join(os.sep, self.rospath, "pkgs", "rosdeb", "bin", "rosbusybox")
        for fn in files:
          if fn == "rosbusybox": continue
          if fn == "cmake": continue
          if fn.find(".backup-") != -1: continue
          if fn.find(".svn") != -1: continue
          if not fn.startswith("ros"): continue
          os.symlink(rosbusybox, os.path.join(self.usrbinpath, fn))

    ## add the ros.pth to the python site-packages
    sitepkgs = os.path.join(self.tmpdir, "usr", "lib", "python2.5", "site-packages")
    makedir(sitepkgs)

    os.symlink(os.path.join(os.sep, self.rospath, "python"), os.path.join(sitepkgs, "ros"))

    ## create the ros module __init__
    os.symlink(os.path.join(os.sep, self.rospath, "pkgs", "roslib", "src", "ros", "__init__.py"), os.path.join(self.pythonpath, "__init__.py"))
    os.symlink(os.path.join(os.sep, self.rospath, "pkgs", "roslib", "src", "roslib"), os.path.join(sitepkgs, "roslib"))

    

    ## compute the size of the package
    duStr = subprocess.Popen(["du", "-bs", self.tmpdir], stdout=subprocess.PIPE).communicate()[0].strip()
    duStrParts = duStr.split()
    size = int(duStrParts[0])

    rosRoot = os.environ["ROS_ROOT"]

    version = "1-" + getSVNVersion(rosRoot) + "-" + self.osdict['DISTRIB_RELEASE']

    ## write the control file
    fp = open(os.path.join(self.debianpath, "control"), "w")
    fp.write("Package: ros-core\n")
    fp.write("Source: ros-core\n")
    fp.write("Version: %s\n" % version)
    fp.write("Architecture: i386\n")
    fp.write("Maintainer: Willow Garage <hassan@willowgarage.com>\n")
    fp.write("Installed-Size: %s\n" % size)
    fp.write("Depends: %s\n" % string.join(packages, ", "))
    fp.write("Section: tools\n")
    fp.write("Priority: optional\n")
    fp.write("Homepage: http://ros.willowgarage.com/prwiki/ros-core\n")
    fp.write("Description: ROS - robot operating system\n")
    fp.write(" text\n")
    fp.write(" text\n")
    fp.write(" .\n")
    fp.write(" text\n")

  def _rosCorePackageList(self):
    environ = os.environ.copy()
    del environ['ROS_PACKAGE_PATH']
    
    fp = subprocess.Popen(["rospack", "list"], stdout=subprocess.PIPE, env=environ).stdout  
    packages = []

    while 1:
      line = fp.readline()
      if not line: break

      parts = line.strip().split(" ")

      #if parts[0] == "roslisp": continue
      packages.append(parts[0])

    return packages

  def _rosCorePackages(self):
    packages = []
    invalidCoreDepends = ['python-wxgtk2.8', 'graphviz', 'python-gtk2-dev', 'libwxgtk2.8-dev', 'python-pydot', 'python-gtk2', 'fakeroot']

    for pkg in self._rosCorePackageList():
      deps = self.getSysDepend(pkg)
      flag = True
      for dep in deps: 
        if dep in invalidCoreDepends: flag = False
      if flag:
        packages.append(pkg)
    return packages

  def rosCorePackages(self):
    packages = self._rosCorePackages()
    dpackages = []
    for package in packages:
      dpackages.append(toDebianName(package))
    return dpackages

  def buildRosCorePackages(self):
    if self.buildFlag:
      environ = os.environ.copy()
      del environ['ROS_PACKAGE_PATH']

      packages = self._rosCorePackages()
      for package in packages:
        pkgpath = getPackagePath(package)
        if os.path.exists(os.path.join(pkgpath, "ROS_NOBUILD")):
          continue
        if os.path.exists(os.path.join(pkgpath, "ROS_BUILD_BLACKLIST")):
          continue
        subprocess.Popen(["rosmake", package], env=environ).communicate()

    for pkg in self._rosCorePackages():
      self.buildTree(pkg)


  def buildTree(self, pkg):
    if _DEBUG: print "buildTree", pkg
    rospath = "usr/lib/ros"

    ## find the path to the package
    pkgpath = getPackagePath(pkg)

    ## find
    fp = subprocess.Popen(["find", pkgpath, "!", "-path", "*/.svn*", "!", "-name", "*.cpp"], stdout=subprocess.PIPE).stdout

    pythonmodules = []

    files = []
    while 1:
      line = fp.readline()
      if not line: break
      line = line.strip()

      fullfn = line
      if os.path.isdir(fullfn): continue
      fn = line[len(pkgpath)+1:]
      _path,_fn = os.path.split(fn)

      f, ext = os.path.splitext(fn)

      if not os.path.exists(fullfn): continue

      fstat = os.stat(fullfn)


      if fn[0] == ".": continue
      elif _path.find("build/lib") == 0:  pass
      elif _path.find("build/") == 0:  continue
      elif _path == "build": continue
      elif fn.find(".backup-") != -1: continue
      elif fn.find(".libs/") != -1:   continue
      elif fn.find("/.svn/") != -1:   continue
      elif fn.find("/distutils/") != -1: continue
      elif _path.find("/test/") != -1:   continue
      elif ext in (".o", ".d", ".c", ".cpp", ".h"): continue

      if pkg == "xenomai":
        if _path.startswith("xenomai-svn/"): continue
        if _path.startswith("linux/"): continue
        if ext in ".bz2": continue
      if pkg == "opencv_latest":
        if _path.startswith("opencv-cvs/"): continue
      if pkg == "gazebo":
        if _path.startswith("gazebo-svn-ogre-v1.4.9/"): continue
      if pkg == "bullet":
        if _path.startswith("bullet_svn/"): continue
      if pkg == "Cg":
        if ext in ".tgz": continue


      if ext == ".so" or fn.find(".so.") != -1:
        tmpfullfn = os.path.join("../pkgs", pkg, fn)
        files.append((fullfn, fn))
        #print os.path.join(self.libpath, _fn)
        if not os.path.islink(os.path.join(self.libpath, _fn)):
          os.symlink(tmpfullfn, os.path.join(self.libpath, _fn))
      elif ext == ".py":
        if fn.endswith("__init__.py"):
          p,f = os.path.split(fn)
          pythonmodules.append(p)
        files.append((fullfn, fn))
      else:
        files.append((fullfn, fn))

    ## copy files to the installation root
    for fullfn, fn in files:
      tmpfullfn = os.path.join(self.pkgspath, pkg, fn)
      path, f1 = os.path.split(tmpfullfn)
      if not os.path.isdir(path): makedir(path)
      #if os.path.islink(fullfn): 
      #  rlink = os.readlink(fullfn)
      subprocess.Popen(["cp", "-a", fullfn, tmpfullfn]).communicate()

      f, ext = os.path.splitext(fn)
      
      if not os.path.islink(tmpfullfn):
        if ext == ".so" or fn.find(".so.") != -1:
          subprocess.Popen(["strip", tmpfullfn]).communicate()
        elif path.endswith("/bin"):
          subprocess.Popen(["strip", tmpfullfn]).communicate()

    if 0:
      ## symlink the python mobules
      moduleDict = {}
      pythonmodules.sort()
      for module in pythonmodules:
        base, fn = os.path.split(module)
        moduleDict[module] = 1
        if base in moduleDict: 
          continue
        tmpfullfn = os.path.join("..", "pkgs", pkg, module)
        if not os.path.exists(os.path.join(self.pythonpath, fn)):
          os.symlink(tmpfullfn, os.path.join(self.pythonpath, fn))

    open(os.path.join(self.pkgspath, pkg, "ROS_NOBUILD"), "w")


  def getDescription(self, manifest):
    description = manifest.description
    if not description:
      description = ""
      return
    description = description.replace("</description>", "")
    i = description.find("<description ")
    if i != -1:
      j = description.find('>')
      description = description[j+1:]

    description = description.strip()
    desc = []
    lines = description.split('\n')
    for line in lines:
      nline = line.strip()
      if not nline:  desc.append(" .")
      else:
        desc.append(" " + line)
    return string.join(desc, "\n")
        
    
    

  def buildControl(self, pkg):
    depstr = subprocess.Popen(["rospack", "deps", pkg], stdout=subprocess.PIPE).communicate()[0]
    deps = []
    if depstr:
      deps = depstr.strip().split("\n")

    ## convert the ros package names into debian package names
    pkgname = toDebianName(pkg)

    packages = self.getSysDepend(pkg)

    #packages.append("libc6")
    packages.append("ros-core")
    for dep in deps:
      ddep = toDebianName(dep)
      if ddep in self.rosCorePackagesDict: continue
      packages.append(ddep)

    ## compute the size of the package
    duStr = subprocess.Popen(["du", "-bs", self.tmpdir], stdout=subprocess.PIPE).communicate()[0].strip()
    duStrParts = duStr.split()
    size = int(duStrParts[0])

    pkgpath = getPackagePath(pkg)

    version = "1-" + getSVNVersion(pkgpath) + "-" + self.osdict['DISTRIB_RELEASE']

    mxml = open(os.path.join(pkgpath, "manifest.xml"), "r").read()
    mani = roslib.manifest.parse(mxml)

    description = self.getDescription(mani)

    ## write the control file
    fp = open(os.path.join(self.debianpath, "control"), "w")
    fp.write("Package: %s\n" % pkgname)
    fp.write("Source: %s\n" % pkgname)
    fp.write("Version: %s\n" % version)
    fp.write("Architecture: i386\n")
    fp.write("Maintainer: Willow Garage <hassan-rosdeb@willowgarage.com>\n")
    fp.write("Installed-Size: %s\n" % size)
    fp.write("Depends: %s\n" % string.join(packages, ", "))
    fp.write("Section: tools\n")
    fp.write("Priority: optional\n")
    if mani.url:
      fp.write("Homepage: %s\n" % mani.url)
    fp.write("Description: ROS - %s\n" % mani.brief)
    fp.write(" %s\n" % description)

    fp.close()

    #print open(os.path.join(self.debianpath, "control"), "r").read()



  def buildDebs(self, pkg, dest_path, version="0.0"):

    roscorepackages = self._rosCorePackages()

    if pkg in roscorepackages:
      print "[rosdeb] Error: cannot build deb from ros-core"
      return

    packages = []

    if pkg == "all":
      packages.append("ros-core")
      str = subprocess.Popen(["rospack", "list"], stdout=subprocess.PIPE).communicate()[0]
      _pkgs = []
      if str: _pkgs = str.split("\n")
      for _line in _pkgs:
        _line = _line.split()
        if len(_line) == 2:
          _pkg = _line[0]
          if _pkg in roscorepackages: continue
          packages.append(_pkg)
    else:
      packages.append(pkg)

      if pkg != "ros-core":
        depstr = subprocess.Popen(["rospack", "deps", pkg], stdout=subprocess.PIPE).communicate()[0]
        deps = []
        if depstr: deps = depstr.strip().split("\n")
        for dep in deps:
          if dep in roscorepackages: continue
          packages.append(dep)
      
    for _pkg in packages:
      if _pkg in ("roslisp", ): continue

      self.buildDebFile(_pkg, dest_path)
      
  def buildDebFile(self, pkg, dest_path):
    #print "buildDebFile", pkg, dest_path
    try:
      self.setupTemporaryDirectory()
      self._buildDebFile(pkg, dest_path)
    finally:
      self.removeTmpDir()
      

  def _buildDebFile(self, pkg, dest_path):
    if pkg == "ros-core":
      pkgpath = os.environ["ROS_ROOT"]
    else:
      pkgpath = getPackagePath(pkg)
    version = "1-" + getSVNVersion(pkgpath) + "-" + self.osdict['DISTRIB_RELEASE']

    arch = self.arch
    debpkg = toDebianName(pkg)

    debfn = os.path.join(dest_path, "%(debpkg)s_%(version)s_%(arch)s.deb" % locals())
    if os.path.exists(debfn): 
      print "Already build: %s" % debfn
      return

    print "Packing %(pkg)s and its dependencies into %(debfn)s..." % locals()

    if os.path.exists(debfn): 
      os.remove(debfn)

    pkgpath = getPackagePath(pkg)
    
    if pkg == "ros-core":
      self.buildRosCore()
      self.buildRosCorePackages()

    else:
      if self.buildFlag:
        if not os.path.exists(os.path.join(pkgpath, "ROS_NOBUILD")) and not os.path.exists(os.path.join(pkgpath, "ROS_BUILD_BLACKLIST")):
          subprocess.Popen(["rosmake", pkg]).communicate()

      self.buildTree(pkg)

      self.buildControl(pkg)

    p = subprocess.Popen(["fakeroot", "dpkg-deb", "--build", self.tmpdir, debfn], stdout=subprocess.PIPE)
    out = p.communicate()[0]
    if p.returncode != 0:
      print 'returncode', p.returncode
      raise DebianBuildError

  def getSysDepend(self, pkg):
    ostype, osversion = getOS()

    fp = subprocess.Popen(["rospack", "sysdeps0", pkg], stdout=subprocess.PIPE).stdout  

    deps = []

    while 1:
      line = fp.readline()
      if not line: break

      line = line.strip()
      parts = line.split("\t")

      d = {}
      d['os'] = ""
      d['version'] = ""

      for part in parts:
        kvparts = part.split(": ", 1)

        key,val = kvparts
        d[key] = val

      if d['os'] == ostype:
        if d['version'].startswith(osversion):
          deps.append(d['package'])
        elif d['version'].startswith("8.04"):
          deps.append(d['package'])
          
    return deps

        

def test():
  pass

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  path, progname = os.path.split(argv[0])
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug", "nobuild", "build"])

  buildFlag = False

  testflag = 0
  if len(args) not in (1,2):
    usage(progname)
    return
  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      _DEBUG = True
    elif field == "--test":
      testflag = 1
    elif field == "--nobuild":
      buildFlag = False
    elif field == "--build":
      buildFlag = True

  if testflag:
    test()
    return

  pkg = args[0]

  cwd = os.getcwd()

  osdict = getOSDict()
  packager = DebianPackager(osdict, arch="i386")
  packager.buildFlag = buildFlag
  packager.buildDebs(pkg, cwd)

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
