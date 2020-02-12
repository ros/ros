#! /usr/bin/env python

# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author Tully Foote/tfoote@willowgarage.com

import os
import sys

import rospkg
import rospkg.os_detect


def _platform_supported(m, os, version):
    for p in m.platforms:
        if os == p.os and version == p.version:
            return True
    return False


def platform_supported(rospack, pkg, os, version):
    """
    Return whether the platform defined by os and version is marked as supported in the package
    @param pkg The package to test for support
    @param os The os name to test for support
    @param version The os version to test for support
    """
    return _platform_supported(rospack.get_manifest(pkg), os, version)


class PackageFlagTracker:
    """This will use the dependency tracker to test if packages are
    blacklisted and all their dependents."""
    def __init__(self, dependency_tracker, os_name=None, os_version=None):
        if not os_name and not os_version:
            try:
                osd = rospkg.os_detect.OsDetect()
                self.os_name = osd.get_codename()
                self.os_version = osd.get_version()
            except rospkg.os_detect.OsNotDetected:
                sys.stderr.write('Could not detect OS. platform detection will not work\n')
        else:
            self.os_name = os_name
            self.os_version = os_version

        self.rospack = rospkg.RosPack()
        self.blacklisted = {}
        self.blacklisted_osx = {}
        self.nobuild = set()
        self.nomakefile = set()
        self.packages_tested = set()
        self.dependency_tracker = dependency_tracker
        self.build_failed = set()

    def register_blacklisted(self, blacklisted_package, dependent_package):
        if dependent_package in self.blacklisted.keys():
            self.blacklisted[dependent_package].append(blacklisted_package)
        else:
            self.blacklisted[dependent_package] = [blacklisted_package]

    def register_blacklisted_osx(self, blacklisted_package, dependent_package):
        if dependent_package in self.blacklisted_osx:
            self.blacklisted_osx[dependent_package].append(blacklisted_package)
        else:
            self.blacklisted_osx[dependent_package] = [blacklisted_package]

    def _check_package_flags(self, package):
        if package in self.packages_tested:
            return
        rospack = self.rospack
        path = rospack.get_path(package)

        if os.path.exists(os.path.join(path, 'ROS_BUILD_BLACKLIST')):
            self.register_blacklisted(package, package)
            for p in rospack.get_depends_on(package, implicit=True):
                self.register_blacklisted(package, p)

        if os.path.exists(os.path.join(path, 'ROS_BUILD_BLACKLIST_OSX')):
            self.register_blacklisted_osx(package, package)
            for p in rospack.get_depends_on(package, implicit=True):
                self.register_blacklisted_osx(package, p)

        # NO_BUILD if marker file or catkin attribute in manifest
        if os.path.exists(os.path.join(path, 'ROS_NOBUILD')):
            self.nobuild.add(package)
        if self.rospack.get_manifest(package).is_catkin:
            self.nobuild.add(package)

        if not os.path.exists(os.path.join(path, 'Makefile')):
            self.nomakefile.add(package)

        self.packages_tested.add(package)

    def is_blacklisted(self, package):
        # this will noop if already run
        self._check_package_flags(package)

        # make sure it's not dependent on a blacklisted package
        for p in self.dependency_tracker.get_deps(package):
            if p not in self.packages_tested:
                self._check_package_flags(p)

        # test result after checking all dependents.
        if package in self.blacklisted:
            return self.blacklisted[package]

        return []

    def is_blacklisted_osx(self, package):
        # this will noop if already run
        self._check_package_flags(package)

        # make sure it's not dependent on a blacklisted_osx package
        for p in self.dependency_tracker.get_deps(package):
            if p not in self.packages_tested:
                self._check_package_flags(p)

        # test result after checking all dependents.
        if package in self.blacklisted_osx:
            return self.blacklisted_osx[package]

        return []

    def has_nobuild(self, package):
        # this will noop if already run
        self._check_package_flags(package)

        # Short circuit if known result
        if package in self.nobuild:
            return True
        return False

    def has_makefile(self, package):
        # this will noop if already run
        self._check_package_flags(package)

        # Short circuit if known result
        if package in self.nomakefile:
            return False
        return True

    def add_nobuild(self, package):
        if self.has_nobuild(package):
            return True
        with open(os.path.join(self.rospack.get_path(package), 'ROS_NOBUILD'), 'w') as f:
            f.write('created by rosmake to mark as installed')
            self.nobuild.add(package)
            return True
        return False

    def remove_nobuild(self, package):
        if not self.has_nobuild(package):
            return True
        try:
            os.remove(os.path.join(self.rospack.get_path(package), 'ROS_NOBUILD'))
            self.nobuild.remove(package)
            return True
        except Exception:
            return False

    def mark_build_failed(self, package):
        self.build_failed.add(package)

    def build_failed(self, package):
        return package in self.build_failed

    def can_build(self, pkg, use_blacklist=False, failed_packages=[], use_makefile=True):
        """
        Return (buildable, error, "reason why not")
        """
        output_str = ''
        output_state = True
        buildable = True

        previously_failed_pkgs = [pk for pk in failed_packages if pk in self.dependency_tracker.get_deps(pkg)]
        if len(previously_failed_pkgs) > 0:
            buildable = False
            output_state = False
            output_str += ' Package %s cannot be built for dependent package(s) %s failed. \n' % (pkg, previously_failed_pkgs)

        if use_blacklist:
            black_listed_dependents = self.is_blacklisted(pkg)
            if len(black_listed_dependents) > 0:
                buildable = False
                output_str += 'Cannot build %s ROS_BUILD_BLACKLIST found in packages %s' % (pkg, black_listed_dependents)

        if self.has_nobuild(pkg):
            buildable = False
            output_state = True  # dependents are ok, it should already be built
            output_str += 'ROS_NOBUILD in package %s\n' % pkg

        if use_makefile and not self.has_makefile(pkg):
            output_state = True  # dependents are ok no need to build
            buildable = False
            output_str += ' No Makefile in package %s\n' % pkg

        if output_str and output_str[-1] == '\n':
            output_str = output_str[:-1]

        return (buildable, output_state, output_str)
