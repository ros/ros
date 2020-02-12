#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#
# Revision $Id$

"""
Generate HTML reports from coverage.py (aka python-coverage). This is
currently a no-frills backend tool.
"""

import sys

import roslib

try:
    import coverage
except ImportError:
    sys.stderr.write("ERROR: cannot import python-coverage, coverage report will not run.\nTo install coverage, run 'easy_install coverage'\n")
    sys.exit(1)


def coverage_html():
    import os.path
    if not os.path.isfile('.coverage-modules'):
        sys.stderr.write('No .coverage-modules file; nothing to do\n')
        return

    with open('.coverage-modules', 'r') as f:
        modules = [x for x in f.read().split('\n') if x.strip()]

    cov = coverage.coverage()
    cov.load()

    # import everything
    for m in modules:
        try:
            base = m.split('.')[0]
            roslib.load_manifest(base)
            __import__(m)
        except Exception:
            sys.stderr.write('WARN: cannot import %s\n' % (base))

    modlist = '\n'.join([' * %s' % m for m in modules])
    sys.stdout.write('Generating for\n%s\n' % (modlist))

    # load the module instances to pass to coverage so it can generate annotation html reports
    mods = []

    # TODO: rewrite, buggy
    for m in modules:
        mods.extend([v for v in sys.modules.values() if v and v.__name__.startswith(m) and v not in mods])

    # dump the output to covhtml directory
    cov.html_report(mods, directory='covhtml')


if __name__ == '__main__':
    coverage_html()
