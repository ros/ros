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
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
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

import hashlib
import os
import sys
import urllib
from optparse import OptionParser

NAME = 'download_checkmd5.py'


def main():
    parser = OptionParser(usage='usage: %prog URI dest [md5sum]', prog=NAME)
    options, args = parser.parse_args()
    md5sum = None
    if len(args) == 2:
        uri, dest = args
    elif len(args) == 3:
        uri, dest, md5sum = args
    else:
        parser.error('wrong number of arguments')

    # Create intermediate directories as necessary, #2970
    d = os.path.dirname(dest)
    if len(d) and not os.path.exists(d):
        os.makedirs(d)

    fresh = False
    if not os.path.exists(dest):
        sys.stdout.write('[rosbuild] Downloading %s to %s...' % (uri, dest))
        sys.stdout.flush()
        urllib.urlretrieve(uri, dest)
        sys.stdout.write('Done\n')
        fresh = True

    if md5sum:
        m = hashlib.md5(open(dest).read())
        d = m.hexdigest()

        print('[rosbuild] Checking md5sum on %s' % (dest))

        if d != md5sum:
            if not fresh:
                print('[rosbuild] WARNING: md5sum mismatch (%s != %s); re-downloading file %s' % (d, md5sum, dest))
                os.remove(dest)

                # Try one more time
                urllib.urlretrieve(uri, dest)
                m = hashlib.md5(open(dest).read())
                d = m.hexdigest()

            if d != md5sum:
                print('[rosbuild] ERROR: md5sum mismatch (%s != %s) on %s; aborting' % (d, md5sum, dest))
                return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
