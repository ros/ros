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

PKG = 'rosbag'
import roslib; roslib.load_manifest(PKG)

import optparse
import os
import shutil
import sys

import rosbag
from rosbag import bag

if __name__ == '__main__':
    parser = optparse.OptionParser(usage='slash.py [options] BAGFILE1 [BAGFILE2 ...]',
                                   description='Slash one or more bag files.')

    (options, args) = parser.parse_args(sys.argv[1:])

    if len(args) < 1:
        parser.error('You must specify at least one bag file.')
        
    for filename in args:
        b = rosbag.Bag(filename)
        index_pos = b._index_data_pos
        b.close()

        (root, ext) = os.path.splitext(filename)
        slash_filename = '%s.slash%s' % (root, ext)

        shutil.copy(filename, slash_filename)
        f = open(slash_filename, 'r+b')
        f.seek(b._file_header_pos)
        header = {
            'op':          bag._pack_uint8(bag._OP_FILE_HEADER),
            'index_pos':   bag._pack_uint64(0),
            'conn_count':  bag._pack_uint32(0),
            'chunk_count': bag._pack_uint32(0)
        }
        bag._write_record(f, header, padded_size=bag._FILE_HEADER_LENGTH)
        f.truncate(index_pos / 100)
        f.close()
        
        print '%s slashed.' % slash_filename
        
        (root, ext) = os.path.splitext(filename)
        reindex_filename = '%s.reindex%s' % (root, ext)
        shutil.copy(slash_filename, reindex_filename)

        bv = rosbag.Bag(slash_filename, allow_unindexed=True)
        version = bv.version
        bv.close()

        if version == 102:
            b = rosbag.Bag(slash_filename, allow_unindexed=True)

            reindexed = rosbag.Bag(reindex_filename, 'w')

            b.reindex()

            try:
                for (topic, msg, t) in b.read_messages():
                    print topic, t
                    reindexed.write(topic, msg, t)
            except:
                pass
            reindexed.close()

            b.close()
        else:
            try:
                b = rosbag.Bag(reindex_filename, 'a', allow_unindexed=True)
            except Exception, ex:
                print str(ex)
            try:
                b.reindex()
            except:
                pass
            b.close()
