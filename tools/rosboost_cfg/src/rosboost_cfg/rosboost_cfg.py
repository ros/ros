#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

from __future__ import print_function

import os
import platform
import sys
from glob import glob
from optparse import OptionParser

lib_suffix = 'so'
if (sys.platform == 'darwin'):
    lib_suffix = 'dylib'

link_static = 'ROS_BOOST_LINK' in os.environ and os.environ['ROS_BOOST_LINK'] == 'static'
if (link_static):
    lib_suffix = 'a'

no_L_or_I = 'ROS_BOOST_NO_L_OR_I' in os.environ

boost_version = None
if ('ROS_BOOST_VERSION' in os.environ and len(os.environ['ROS_BOOST_VERSION']) > 0):
    ver = os.environ['ROS_BOOST_VERSION']
    ver = ver.split('.')

    boost_version = [int(v) for v in ver]
    if (len(boost_version) == 2):
        boost_version.append(0)


def print_usage_and_exit():
    print('Usage: rosboost-cfg --lflags [thread,regex,graph,...]')
    print('       rosboost-cfg --cflags')
    print('       rosboost-cfg --libs [thread,regex,graph,...]')
    print('       rosboost-cfg --include_dirs')
    print('       rosboost-cfg --lib_dirs')
    print('       rosboost-cfg --root')
    sys.exit(1)


class BoostError(Exception):

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class Version(object):

    def __init__(self, major, minor, patch, root, include_dir, lib_dir, is_default_search_location):
        self.major = major
        self.minor = minor
        self.patch = patch
        self.root = root
        self.include_dir = include_dir
        self.lib_dir = lib_dir
        self.is_default_search_location = is_default_search_location
        self.is_system_install = os.path.split(self.include_dir)[0] == self.root

    def __cmp__(self, other):
        if (self.major != other.major):
            if self.major < other.major:
                return -1
            else:
                return 1
        if (self.minor != other.minor):
            if self.minor < other.minor:
                return -1
            else:
                return 1
        if (self.patch != other.patch):
            if self.patch < other.patch:
                return -1
            else:
                return 1

        return 0

    def __repr__(self):
        return repr((self.major, self.minor, self.patch, self.root, self.include_dir, self.is_default_search_location, self.is_system_install))


def find_lib_dir(root_dir, multiarch=''):
    # prefer lib64 unless explicitly specified in the environment
    if ('ROS_BOOST_LIB_DIR_NAME' in os.environ):
        possible_dirs = [os.path.join(root_dir, os.environ['ROS_BOOST_LIB_DIR_NAME'])]
    else:
        possible_dirs = [os.path.join(root_dir, 'lib64'), os.path.join(root_dir, 'lib')]
        if multiarch:
            possible_dirs = [os.path.join(root_dir, 'lib/%s' % multiarch)] + possible_dirs

    for p in possible_dirs:
        glob_files = glob('%s*' % (os.path.join(p, 'libboost*')))
        if (len(glob_files) > 0):
            return p

    return None


def extract_versions(dir, is_default_search_location, multiarch=''):
    version_paths = [os.path.join(dir, 'version.hpp'),
                     os.path.join(dir, 'boost', 'version.hpp')]
    glob_dirs = glob('%s*' % (os.path.join(dir, 'boost-')))
    [version_paths.append(os.path.join(gdir, 'boost', 'version.hpp')) for gdir in glob_dirs]

    versions = []

    for p in version_paths:
        ver_string = ''
        if (os.path.isfile(p)):
            fh = open(p, 'r')
            lines = fh.readlines()
            fh.close()
            for line in lines:
                if line.find('#define BOOST_VERSION ') > -1:
                    def_string = line.split()
                    ver_string = def_string[2]
                    ver_int = int(ver_string)
                    patch = ver_int % 100
                    minor = ver_int / 100 % 1000
                    major = ver_int / 100000
                    include_dir = os.path.split(os.path.split(p)[0])[0]
                    root_dir = os.path.split(dir)[0]
                    lib_dir = find_lib_dir(root_dir, multiarch)
                    versions.append(Version(major, minor, patch, root_dir, include_dir, lib_dir, is_default_search_location))

    return versions


def find_versions(search_paths, multiarch=''):
    vers = []

    for path, system in search_paths:
        path = os.path.join(path, 'include')
        pvers = extract_versions(path, system, multiarch)
        [vers.append(ver) for ver in pvers]

    if (len(vers) == 0):
        return None

    if (boost_version is not None):
        for v in vers:
            if (v.major == boost_version[0] and v.minor == boost_version[1] and v.patch == boost_version[2]):
                return [v]

        raise BoostError('Could not find boost version %s required by ROS_BOOST_VERSION environment variable' % (boost_version))

    vers.sort()
    return vers


def find_boost(search_paths, multiarch=''):
    result = find_versions(search_paths, multiarch)
    if result is None:
        return None
    if len(result) > 1:
        sys.stderr.write("WARN, found multiple boost versions '%s', using latest" % result)
    return result[-1]


def search_paths(sysroot):
    _search_paths = [(sysroot+'/usr', True),
                     (sysroot+'/usr/local', True),
                     (None if 'INCLUDE_DIRS' not in os.environ else os.environ['INCLUDE_DIRS'], True),
                     (None if 'CPATH' not in os.environ else os.environ['CPATH'], True),
                     (None if 'C_INCLUDE_PATH' not in os.environ else os.environ['C_INCLUDE_PATH'], True),
                     (None if 'CPLUS_INCLUDE_PATH' not in os.environ else os.environ['CPLUS_INCLUDE_PATH'], True),
                     (None if 'ROS_BOOST_ROOT' not in os.environ else os.environ['ROS_BOOST_ROOT'], False)]

    search_paths = []
    for (str, system) in _search_paths:
        if (str is not None):
            dirs = str.split(':')
            for dir in dirs:
                if (len(dir) > 0):
                    if (dir.endswith('/include')):
                        dir = dir[:-len('/include')]
                    search_paths.append((dir, system))
    return search_paths


def lib_dir(ver):
    return ver.lib_dir


def find_lib(ver, name, full_lib=link_static):
    global lib_suffix
    global link_static

    dynamic_search_paths = []
    static_search_paths = []

    if (ver.is_system_install):
        dynamic_search_paths = ['libboost_%s-mt.%s' % (name, lib_suffix),
                                'libboost_%s.%s' % (name, lib_suffix)]
        static_search_paths = ['libboost_%s-mt.a' % (name),
                               'libboost_%s.a' % (name)]
    else:
        dynamic_search_paths = ['libboost_%s*%s_%s*.%s' % (name, ver.major, ver.minor, lib_suffix),
                                'libboost_%s-mt*.%s' % (name, lib_suffix),
                                'libboost_%s*.%s' % (name, lib_suffix)]
        static_search_paths = ['libboost_%s*%s_%s*.a' % (name, ver.major, ver.minor),
                               'libboost_%s-mt*.a' % (name),
                               'libboost_%s*.a' % (name)]

    # Boost.Python needs some special handling on some systems (Karmic), since it may have per-python-version libs
    if (name == 'python'):
        python_ver = platform.python_version().split('.')
        dynamic_search_paths = ['libboost_%s-mt-py%s%s.%s' % (name, python_ver[0], python_ver[1], lib_suffix),
                                'libboost_%s-py%s%s.%s' % (name, python_ver[0], python_ver[1], lib_suffix)] + dynamic_search_paths
        static_search_paths = ['libboost_%s-mt-py%s%s.a' % (name, python_ver[0], python_ver[1]),
                               'libboost_%s-py%s%s.a' % (name, python_ver[0], python_ver[1])] + static_search_paths

    search_paths = static_search_paths if link_static else dynamic_search_paths

    dir = lib_dir(ver)

    if dir is None:
        raise BoostError('Could not locate library [%s], version %s' % (name, ver))

    for p in search_paths:
        globstr = os.path.join(dir, p)
        libs = glob(globstr)
        if (len(libs) > 0):
            if (full_lib):
                return libs[0]
            else:
                return os.path.basename(libs[0])

    raise BoostError('Could not locate library [%s], version %s in lib directory [%s]' % (name, ver, dir))


def include_dirs(ver, prefix=''):
    if ver.is_system_install or no_L_or_I:
        return ''

    return ' %s%s' % (prefix, ver.include_dir)


def cflags(ver):
    return include_dirs(ver, '-I')


def lib_dir_flags(ver):
    if not ver.is_default_search_location:
        dir = lib_dir(ver)
        return ' -L%s -Wl,-rpath,%s' % (dir, dir)

    return ''


def lib_flags(ver, name):
    lib = find_lib(ver, name)
    if (link_static):
        return ' %s' % (lib)
    else:
        # Cut off "lib" and extension (.so/.a/.dylib/etc.)
        return ' -l%s' % (os.path.splitext(lib)[0][len('lib'):])


def lflags(ver, libs):
    s = lib_dir_flags(ver) + ' '
    for lib in libs:
        s += lib_flags(ver, lib) + ' '
    return s


def libs(ver, libs):
    s = ''
    for lib in libs:
        s += find_lib(ver, lib, True) + ' '
    return s


def lib_dirs(ver):
    if (ver.is_default_search_location or no_L_or_I):
        return ''

    return lib_dir(ver)


OPTIONS = ['libs', 'include_dirs', 'lib_dirs', 'cflags', 'lflags', 'root', 'print_versions', 'version']


def check_one_option(options, key):
    for k in dir(options):
        if (k in OPTIONS):
            v = getattr(options, k)
            if (k != key and v):
                raise BoostError('Only one option (excepting sysroot) is allowed at a time')


def main():
    if (len(sys.argv) < 2):
        print_usage_and_exit()

    parser = OptionParser()
    parser.add_option('-l', '--libs', dest='libs', type='string', help='')
    parser.add_option('-i', '--include_dirs', dest='include_dirs', action='store_true', default=False, help='')
    parser.add_option('-d', '--lib_dirs', dest='lib_dirs', action='store_true', help='')
    parser.add_option('-c', '--cflags', dest='cflags', action='store_true', default=False, help='')
    parser.add_option('-f', '--lflags', dest='lflags', type='string', help='')
    parser.add_option('-r', '--root', dest='root', action='store_true', default=False, help='')
    parser.add_option('-p', '--print_versions', dest='print_versions', action='store_true', default=False, help='')
    parser.add_option('-v', '--version', dest='version', action='store_true', default=False, help='')
    parser.add_option('-s', '--sysroot', dest='sysroot', type='string', default='', help='Location of the system root (usually toolchain root).')
    parser.add_option('-m', '--multiarch', dest='multiarch', type='string', default='', help="Name of multiarch to search below 'lib' folder for libraries.")

    (options, args) = parser.parse_args()

    if (options.print_versions):
        check_one_option(options, 'print_versions')
        for ver in find_versions(search_paths(options.sysroot), options.multiarch):
            print('%s.%s.%s root=%s include_dir=%s' % (ver.major, ver.minor, ver.patch, ver.root, ver.include_dir))
        return

    ver = find_boost(search_paths(options.sysroot), options.multiarch)

    if ver is None:
        raise BoostError('Cannot find boost in any of %s' % search_paths(options.sysroot))
        sys.exit(0)

    if options.version:
        check_one_option(options, 'version')
        print('%s.%s.%s root=%s include_dir=%s' % (ver.major, ver.minor, ver.patch, ver.root, ver.include_dir))
        return

    if ver.major < 1 or (ver.major == 1 and ver.minor < 37):
        raise BoostError('Boost version %s.%s.%s does not meet the minimum requirements of boost 1.37.0' % (ver.major, ver.minor, ver.patch))

    output = ''
    if (options.root):
        check_one_option(options, 'root')
        output = ver.root
    elif (options.libs):
        check_one_option(options, 'libs')
        output = libs(ver, options.libs.split(','))
    elif (options.include_dirs):
        check_one_option(options, 'include_dirs')
        output = include_dirs(ver)
    elif (options.lib_dirs):
        check_one_option(options, 'lib_dirs')
        output = lib_dirs(ver)
    elif (options.cflags):
        check_one_option(options, 'cflags')
        output = cflags(ver)
    elif (options.lflags):
        check_one_option(options, 'lflags')
        output = lflags(ver, options.lflags.split(','))
    else:
        print_usage_and_exit()

    print(output.strip())


if __name__ == '__main__':
    main()
