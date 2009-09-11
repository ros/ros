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

PKG = 'rosbagmigration'
import roslib; roslib.load_manifest(PKG)

from rosbagmigration.rules import MessageMigrator
import rosrecord

import copy

_mm_rule_files = []
_mm = None


## Check whether a bag file can be played in the current system
#
# @param inbag Name of the bag to be checked.
# @param rule_files (optional) extra rule files to use when performing check
# @returns The list of rules that must be created, or None if the bag
#  is up to date.  Note: The empty list [] means the bag must be
#  upgraded, but all the necessary rules already exist
def checkbag(inbag, rule_files=[]):
    global _mm
    global _mm_rule_files
    if (_mm is None or rule_files != _mm_rule_files):
        _mm_rule_files = []
        _mm_rule_files.extend(rule_files)
        _mm = MessageMigrator(_mm_rule_files)

    mm = copy.deepcopy(_mm)

    needs_migration = False
    for i,(topic, msg, t) in enumerate(rosrecord.logplayer(inbag, raw=True)):
        target = mm.find_target(msg[4])
        if (mm.find_path(msg[4], target) != [] or target._type != msg[4]._type):
            needs_migration = True

    if mm.all_rules_valid():
        if needs_migration:
            return []
        else:
            return None
    else:
        return mm.get_invalid_rules()

## Fix a bag so that it can be played in the current system
#
# @param inbag Name of the bag to be fixed.
# @param outbag Name of the bag to be saved.
# @param rule_files (optional) extra rule files to use when performing fix.
# @returns True if migration was successful.
def fixbag(inbag, outbag, rule_files=[]):
    global _mm
    global _mm_rule_files
    if (_mm is None or rule_files != _mm_rule_files):
        _mm_rule_files = []
        _mm_rule_files.extend(rule_files)
        _mm = MessageMigrator(_mm_rule_files)

    mm = copy.deepcopy(_mm)

    # Double parsing the bag is inefficient, but we can speed this up later
    for i,(topic, msg, t) in enumerate(rosrecord.logplayer(inbag, raw=True)):
        target = mm.find_target(msg[4])
        mm.find_path(msg[4], target)

    # Deserializing all messages is inefficient, but we can speed this up later
    if mm.all_rules_valid():
        rebag = rosrecord.Rebagger(outbag)
        for i,(topic, msg, t) in enumerate(rosrecord.logplayer(inbag)):
            new_msg = mm.find_target(msg.__class__)()
            mm.migrate(msg, new_msg)
            rebag.add(topic, new_msg, t)
        rebag.close()    
        return True
    else:
        return False

