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
#
# Revision $Id$

PKG = 'rosbag'
import roslib; roslib.load_manifest(PKG)

import optparse
import os
import sys

import rosbag.migration

def print_trans(old, new, indent):
    from_txt = '%s [%s]' % (old._type, old._md5sum)
    if new is not None:
        to_txt= '%s [%s]' % (new._type, new._md5sum)
    else:
        to_txt = 'Unknown'
    print '    ' * indent + ' * From: %s' % from_txt
    print '    ' * indent + '   To:   %s' % to_txt

if __name__ == '__main__':
    parser = optparse.OptionParser(usage='usage: checkbag.py [-g] <inbag> [rulefile1, rulefile2, ...]')
    parser.add_option('-g', '--genrules',  action='store',      dest='rulefile', default=None)
    parser.add_option('-a', '--append',    action='store_true', dest='append')
    parser.add_option('-n', '--noplugins', action='store_true', dest='noplugins')
    (options, args) = parser.parse_args()

    if len(args) < 1:
        parser.error('Incorrect number of arguments')

    if options.append and options.rulefile is None:
        print >> sys.stderr, "Cannot specify -a without also specifying -g"
        exit(2)

    if options.rulefile is not None:
        if os.path.isfile(options.rulefile) and not options.append:
            print >> sys.stderr, "The file %s already exists.  Include -a if you intend to append."%options.rulefile
            exit(2)
        if not os.path.isfile(options.rulefile) and options.append:
            print >> sys.stderr, "The file %s does not exist, and so -a is invalid."%options.rulefile
            exit(2)

    if options.append:
        append_rule = [options.rulefile]
    else:
        append_rule = []

    mm = rosbag.migration.MessageMigrator(args[1:] + append_rule, not options.noplugins)

    migrations = rosbag.migration.checkbag(mm, args[0])
    if len(migrations) == 0:
        print 'Bag file is up to date.'
        exit(0)
        
    print 'The following migrations need to occur:'
    all_rules = []
    for m in migrations:
        all_rules.extend(m[1])

        print_trans(m[0][0].old_class, m[0][-1].new_class, 0)
        if len(m[1]) > 0:
            print "    %d rules missing:" % len(m[1])
            for r in m[1]:
                print_trans(r.old_class, r.new_class, 1)

    if options.rulefile is None:
        if all_rules == []:
            print "\nAll rules defined.  Bag is ready to be migrated"
        else:
            print "\nTo generate rules, please run with -g <rulefile>"
        return

    output = ''
    rules_left = mm.filter_rules_unique(all_rules)

    if rules_left == []:
        print "\nNo additional rule files needed to be generated.  %s not created."%(options.rulefile)
        exit(0)

    while len(rules_left) > 0:
        extra_rules = []
        for r in rules_left:
            if r.new_class is None:
                print 'The message type %s appears to have moved.  Please enter the type to migrate it to.' % r.old_class._type
                new_type = raw_input('>')
                new_class = roslib.message.get_message_class(new_type)
                while new_class is None:
                    print "\'%s\' could not be found in your system.  Please make sure it is built." % new_type
                    new_type = raw_input('>')
                    new_class = roslib.message.get_message_class(new_type)
                new_rule = mm.make_update_rule(r.old_class, new_class)
                R = new_rule(mm, 'GENERATED.' + new_rule.__name__)
                R.find_sub_paths()
                new_rules = [r for r in mm.expand_rules(R.sub_rules) if r.valid == False]
                extra_rules.extend(new_rules)
                print 'Creating the migration rule for %s requires additional missing rules:' % new_type
                for nr in new_rules:
                    print_trans(nr.old_class, nr.new_class,1)
                output += R.get_class_def()
            else:
                output += r.get_class_def()
        rules_left = mm.filter_rules_unique(extra_rules)
    f = open(options.rulefile, 'a')
    f.write(output)
    f.close()

    print '\nThe necessary rule files have been written to: %s' % options.rulefile
