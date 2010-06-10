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

import collections
import copy
from cStringIO import StringIO
import inspect
import itertools
import os
import string
import sys

import roslib.rospack
import roslib.message
import roslib.msgs

import rosbag

# Anything outside the scope of these primitives is a submessage
#_PRIMITIVES = ['bool', 'byte','int8','int16','int32','int64','char','uint8','uint16','uint32','uint64','float32','float64','string','time']

class BagMigrationException(Exception):
    pass

def checkbag(migrator, inbag):
    """
    Check whether a bag file can be played in the current system.
    @param migrator: message migrator to use
    @param inbag name of the bag to be checked.
    @returns A list of tuples for each type in the bag file.  The first
    element of each tuple is the full migration path for the type.  The
    second element of the tuple is the expanded list of invalid rules
    for that particular path.
    """
    checked = set()
    migrations = []

    bag = rosbag.Bag(inbag, 'r')

    for topic, msg, t in bag.read_messages(raw=True):
        key = get_message_key(msg[4])
        if key not in checked:
            target = migrator.find_target(msg[4])
            # Even in the case of a zero-length path (matching md5sums), we still want
            # to migrate in the event of a type change (message move).
            path = migrator.find_path(msg[4], target)
            if len(path) > 0:
                migrations.append((path, [r for r in migrator.expand_rules([sn.rule for sn in path]) if r.valid == False]))

            checked.add(key)

    bag.close()
            
    return migrations

def checkmessages(migrator, messages):
    """
    Check whether a bag file can be played in the current system.
    @param migrator The message migrator to use
    @param message_list A list of message classes.
    @returns A list of tuples for each type in the bag file.  The first
    element of each tuple is the full migration path for the type.  The
    second element of the tuple is the expanded list of invalid rules
    for that particular path.
    """
    
    checked = set()
    migrations = []

    for msg in messages:
        key = get_message_key(msg)
        if key not in checked:
            target = migrator.find_target(msg)
            # Even in the case of a zero-length path (matching md5sums), we still want
            # to migrate in the event of a type change (message move).
            path = migrator.find_path(msg, target)
            if len(path) > 0:
                migrations.append((path, [r for r in migrator.expand_rules([sn.rule for sn in path]) if r.valid == False]))

            checked.add(key)
            
    return migrations

## Fix a bag so that it can be played in the current system
#
# @param migrator The message migrator to use
# @param inbag Name of the bag to be fixed.
# @param outbag Name of the bag to be saved.
# @returns True if migration was successful.
def fixbag(migrator, inbag, outbag):
    # This checks/builds up rules for the given migrator
    res = checkbag(migrator, inbag)

    # Deserializing all messages is inefficient, but we can speed this up later
    if not False in [m[1] == [] for m in res]:
        bag = rosbag.Bag(inbag, 'r')
        rebag = rosbag.Bag(outbag, 'w', options=bag.options)
        for topic, msg, t in bag.read_messages():
            new_msg = migrator.find_target(msg.__class__)()
            migrator.migrate(msg, new_msg)
            rebag.write(topic, new_msg, t)
        rebag.close()
        bag.close()
        return True
    else:
        return False

## Fix a bag so that it can be played in the current system
#
# @param migrator The message migrator to use
# @param inbag Name of the bag to be fixed.
# @param outbag Name of the bag to be saved.
# @returns [] if bag could be migrated, otherwise, it returns the list of necessary migration paths
def fixbag2(migrator, inbag, outbag):
    # This checks/builds up rules for the given migrator
    res = checkbag(migrator, inbag)

    migrations = [m for m in res if len(m[1]) > 0]

    # Deserializing all messages is inefficient, but we can speed this up later
    if len(migrations) == 0:
        bag = rosbag.Bag(inbag, 'r')
        rebag = rosbag.Bag(outbag, 'w', options=bag.options)
        for topic, msg, t in bag.read_messages():
            new_msg = migrator.find_target(msg.__class__)()
            migrator.migrate(msg, new_msg)
            rebag.write(topic, new_msg, t)
        rebag.close()
        bag.close()

    return migrations

## Helper function to strip out roslib and package name from name usages.
# 
# There is some inconsistency in whether a fully-qualified path is
# used for sub-messages within a given message.  This function is
# useful for stripping out the package name in a fully qualified
# sub-message.
#
# @param name      The name to clean.
# @param top_name  The name of the top-level type
# @returns         The cleaned version of the name.
def clean_name(name, top_name):
    name_split = name.split('/')
    try:
        name_split.remove('roslib')
    except ValueError:
        pass
    try:
        name_split.remove(top_name.split('/')[0])
    except ValueError:
        pass
    new_name = string.join(name_split,'/')
    return new_name

## Helper function to ensure we end up with a qualified name
# 
# There is some inconsistency in whether a fully-qualified path is
# used for sub-messages within a given message.  This function is
# useful for ensuring that a name is fully qualified correctly.
#
# @param name      The name to quailfy
# @param top_name  The name of the top-level type
# @returns         The qualified version of the name.
def qualified_name(name, top_name):
    # First clean the name, to make everyting else more deterministic
    tmp_name = clean_name(name, top_name)

    if len(tmp_name.split('/')) == 2 or (roslib.msgs.is_builtin(tmp_name)):
        return tmp_name
    elif tmp_name == 'Header':
        return 'roslib/Header'
    else:
        return top_name.split('/')[0] + '/' + tmp_name

## Helper function to return a key from a given class
#
# For now, we choose the tuple (type,md5sum) as a unique key for the
# class.  However, this is subject to change and assumptions about keys
# should not be made other than their uniqueness.
#
# @param c  The message class or instance to get a key for
# @returns The unique key
def get_message_key(c):
    try:
        return (c._type, c._md5sum)
    except:
        return None

## Helper function to return a key for a given path
#
# For now, we choose the tuple ((type1,md5sum1),(type2,md5sum2)) as a
# unique key for the path.  However, this is subject to change and
# assumptions about keys should not be made other than their
# uniqueness.
#
# @param c1  The start point of the path
# @param c1  The stop point of the path
# @returns The unique key
def get_path_key(c1, c2):
    try:
        return (get_message_key(c1), get_message_key(c2))
    except:
        return None

## Base class for all message update rules
class MessageUpdateRule(object):
    old_type = ''
    old_full_text = ''
    new_type = ''
    new_full_text = ''
    migrated_types = []

    order = -1

    valid = False

    ## Initialize class
    def __init__(self, migrator, location):
        # Every rule needs to hang onto the migrator so we can potentially use it
        self.migrator = migrator
        self.location = location

        if (self.old_type != self.new_type):
            self.rename_rule = True
        else:
            self.rename_rule = False

        # Instantiate types dynamically based on definition
        try:
            if self.old_type == "":
                raise Exception
            self.old_types = roslib.genpy.generate_dynamic(self.old_type, self.old_full_text)
            self.old_class = self.old_types[self.old_type]
            self.old_md5sum = self.old_class._md5sum
        except:
            self.old_types = []
            self.old_class = None
            self.old_md5sum = ""

        try:
            if self.new_type == "":
                raise Exception
            self.new_types = roslib.genpy.generate_dynamic(self.new_type, self.new_full_text)
            self.new_class = self.new_types[self.new_type]
            self.new_md5sum = self.new_class._md5sum
        except:
            self.new_types = []
            self.new_class = None
            self.new_md5sum = ""

        # We have not populated our sub rules (and ideally should
        # wait until the full scaffold exists before doing this)
        self.sub_rules_done = False
        self.sub_rules_valid = False
        self.sub_rules = []

    ## Find all of the sub paths
    # 
    # For any migrated type the user might want to use, we must make
    # sure the migrator had found a path for it.  To facilitated this
    # check we require that all migrated types must be listed as pairs
    # in the migrated_types field.
    #
    # It would be nice not to need these through performing some kind
    # of other inspection of the update rule itself.
    def find_sub_paths(self):
        self.sub_rules_valid = True
        for (t1, t2) in self.migrated_types:
            try:
                tmp_old_class = self.get_old_class(t1)
            except KeyError:
                print >> sys.stderr, "WARNING: Within rule [%s], specified migrated type [%s] not found in old message types"%(self.location,t1)
                self.sub_rules_valid = False
                continue
            try:
                tmp_new_class = self.get_new_class(t2)
            except KeyError:
                print >> sys.stderr, "WARNING: Within rule [%s], specified migrated type [%s] not found in new message types"%(self.location,t2)
                self.sub_rules_valid = False
                continue

            # If a rule instantiates itself as a subrule (because the
            # author knows the md5sums match), we don't Want to end up
            # with an infinite recursion.
            if (get_message_key(tmp_old_class) != get_message_key(self.old_class)) or (get_message_key(tmp_new_class) != get_message_key(self.new_class)):
                path = self.migrator.find_path(tmp_old_class, tmp_new_class)
                rules = [sn.rule for sn in path]
                self.sub_rules.extend(rules)

            if False in [r.valid for r in rules]:
#                print >> sys.stderr, "WARNING: Within rule [%s] cannot migrate from subtype [%s] to [%s].."%(
#                    self.location, t1, t2)
                self.sub_rules_valid = False
                continue
        self.sub_rules = self.migrator.filter_rules_unique(self.sub_rules)
        self.sub_rules_done = True

    ## Helper function to get the class of a submsg for the new type
    #
    # This function should be used inside of update to access new classes.
    #
    # @param t The subtype to return the class of
    # @returns The class of the new sub type
    def get_new_class(self,t):
        try:
            try:
                return self.new_types[t]
            except KeyError:                
                return self.new_types['roslib/' + t]
        except KeyError:
            return self.new_types[self.new_type.split('/')[0] + '/' + t]

    ## Helper function to get the class of a submsg for the old type
    #
    # This function should be used inside of update to access old classes.
    #
    # @param t The subtype to return the class of
    # @returns The class of the old sub type
    def get_old_class(self,t):
        try:
            try:
                return self.old_types[t]
            except KeyError:                
                return self.old_types['roslib/' + t]
        except KeyError:
            return self.old_types[self.old_type.split('/')[0] + '/' + t]

    ## Actually migrate one sub_type to another
    #
    # This function should be used inside of update to migrate sub msgs.
    #
    # @param msg_from A message instance of the old message type
    # @param msg_to   A message instance of a new message type to be populated
    def migrate(self, msg_from, msg_to):
        tmp_msg_from = clean_name(msg_from._type, self.old_type)
        tmp_msg_to = clean_name(msg_to._type, self.new_type)
        if (tmp_msg_from, tmp_msg_to) not in self.migrated_types:
            raise BagMigrationException("Rule [%s] tried to perform a migration from old [%s] to new [%s] not listed in migrated_types"%(self.location, tmp_msg_from, tmp_msg_to))
        self.migrator.migrate(msg_from, msg_to)

    ## Helper function to migrate a whole array of messages
    #
    # This function should be used inside of update to migrate arrays of sub msgs.
    #
    # @param msg_from_array An array of messages of the old message type
    # @param msg_to_array An array of messages of the new message type (this will be emptied if not already)
    # @param msg_to_class  The name of the new message type since msg_to_array may be an empty array.
    def migrate_array(self, msg_from_array, msg_to_array, msg_to_name):
        msg_to_class = self.get_new_class(msg_to_name)

        while len(msg_to_array) > 0:
            msg_to_array.pop()

        if (len(msg_from_array) == 0):
            return

        tmp_msg_from = clean_name(msg_from_array[0]._type, self.old_type)
        tmp_msg_to = clean_name(msg_to_class._type, self.new_type)
        if (tmp_msg_from, tmp_msg_to) not in self.migrated_types:
            raise BagMigrationException("Rule [%s] tried to perform a migration from old [%s] to new [%s] not listed in migrated_types"%(self.location, tmp_msg_from, tmp_msg_to))

        msg_to_array.extend( [msg_to_class() for i in xrange(len(msg_from_array))] )

        self.migrator.migrate_array(msg_from_array, msg_to_array)

    ## A helper function to print out the definiton of autogenerated messages.
    def get_class_def(self):
        pass

    ## The function actually called by the message migrator 
    #
    # @param old_msg An instance of the old message type.
    # @returns An instance of a new message type
    def apply(self, old_msg):
        if not self.valid:
            raise BagMigrationException("Attempted to apply an invalid rule")
        if not self.sub_rules_done:
            raise BagMigrationException("Attempted to apply a rule without building up its sub rules")
        if not self.sub_rules_valid:
            raise BagMigrationException("Attempted to apply a rule without valid sub-rules")
        if (get_message_key(old_msg) != get_message_key(self.old_class)):
            raise BagMigrationException("Attempted to apply rule to incorrect class %s %s."%(get_message_key(old_msg),get_message_key(self.old_class)))

        # Apply update rule
        new_msg = self.new_class()
        self.update(old_msg, new_msg)

        return new_msg
    
    ## The function which a user overrides to actually perform the message update
    #
    # @param msg_from A message instance of the old message type
    # @param msg_to   A message instance of a new message type to be populated
    def update(self, old_msg, new_msg):
        raise BagMigrationException("Tried to use rule without update overidden")


## A class for book-keeping about rule-chains.
#
# Rule chains define the ordered set of update rules, indexed by
# typename, terminated by a rename rule.  This class is only used
# temporarily to help us get the ordering right, until all explicit
# rules have been loaded (possibly out of order) and the proper
# scaffold can be built.
class RuleChain(object):
    def __init__(self):
        self.chain = []
        self.order_keys = set()
        self.rename = None
 

## A class for arranging the ordered rules
#
# They provide a scaffolding (essentially a linked list) over which we
# assume we can migrate messages forward.  This allows us to verify a
# path exists before actually creating all of the necessary implicit
# rules (mostly migration of sub-messages) that such a path
# necessitates.
class ScaffoldNode(object):
    def __init__(self, old_class, new_class, rule):
        self.old_class = old_class
        self.new_class = new_class
        self.rule = rule
        self.next = None

## A class to actually migrate messages
#
# This is the big class that actually handles all of the fancy
# migration work.  Better documentation to come later.
class MessageMigrator(object):
    def __init__(self, input_rule_files=[], plugins=True):
        # We use the rulechains to scaffold our initial creation of
        # implicit rules.  Each RuleChain is keyed off of a type and
        # consists of an ordered set of update rules followed by an
        # optional rename rule.  For the system rule definitions to be
        # valid, all members of a rulechains must be connectable via
        # implicit rules and all rulechains must terminate in a known
        # system type which is also reachable by an implicit rule.
        self.rulechains = collections.defaultdict(RuleChain)
        
        # The list of all nodes that we can iterate through in the
        # future when making sure all rules have been constructed.
        self.base_nodes = []

        # The list of extra (non-scaffolded) nodes that we can use
        # when determining if all rules are valid and printing invalid
        # rules.
        self.extra_nodes = []

        # A map from typename to the first node of a particular type
        self.first_type = {}
                
        # A map from a typename to all other typenames for which
        # rename rules exist.  This is necessary to determine whether
        # an appropriate implicit rule can actually be constructed.
        self.rename_map = {}

        # The cached set of all found paths, keyed by:
        # ((old_type, old_md5), (new_type, new_md5))
        self.found_paths = {}
        self.found_targets = {}

        # Temporary list of the terminal nodes
        terminal_nodes = []

        # Temporary list of rule modules we are loading
        rule_dicts = []

        self.false_rule_loaded = False
        
        # To make debugging easy we can pass in a list of local
        # rulefiles.
        for r in input_rule_files:
            try:
                scratch_locals = {'MessageUpdateRule':MessageUpdateRule}
                execfile(r,scratch_locals)
                rule_dicts.append((scratch_locals, r))
            except:
                print >> sys.stderr, "Cannot load rule file [%s] in local package"%r

        # Alternatively the preferred method is to load definitions
        # from the migration ruleset export flag.
        if plugins:
            for pkg in roslib.rospack.rospack_depends_on_1('rosbagmigration'):
                m_file = roslib.manifest.manifest_file(pkg, True)
                m = roslib.manifest.parse_file(m_file)
                p_rules = m.get_export('rosbagmigration', 'rule_file')
                roslib.load_manifest(pkg)
                pkg_dir = roslib.packages.get_pkg_dir(pkg)
                for r in p_rules:
                    try:
                        scratch_locals = {'MessageUpdateRule':MessageUpdateRule}
                        execfile(pkg_dir + "/" + r,scratch_locals)
                        rule_dicts.append((scratch_locals, r))
                    except ImportError:
                        print >> sys.stderr, "Cannot load rule file [%s] in package [%s]"%(r, pkg)


        for (rule_dict, location_base) in rule_dicts:
            for (n,c) in rule_dict.iteritems():
                if inspect.isclass(c):
                    if (not c == MessageUpdateRule) and issubclass(c, MessageUpdateRule):
                        self.add_update_rule(c(self, location_base + ':' + n))
                
        if self.false_rule_loaded:
            raise BagMigrationException("Cannot instantiate MessageMigrator with invalid rules")

        # Now, go through and build up a better scaffolded
        # representation, deferring implicit rule generation until
        # complete, since the implicit rule generation and sub-rule
        # population makes use of the scaffold.

        # First we each particular type chain (now including implicit
        # rules).  Additionally, we build up our name remapping lists.


        # For Each rulechain
        for (type,rulechain) in self.rulechains.iteritems():
            first = True
            sn = None
            prev_sn = None

            # Find name remapping list
            rename_set = set([type])
            tmp = rulechain.rename
            while tmp:
                rename_set.add(tmp.new_type)
                if (self.rulechains.has_key(tmp.new_type)):
                    tmp = self.rulechains[tmp.new_type].rename
                else:
                    break
                    
            self.rename_map[type] = rename_set

            # For each element in the rulechain chain, 
            for r in rulechain.chain:
                # Create a scaffoldnode
                sn = ScaffoldNode(r.old_class, r.new_class, r)
                self.base_nodes.append(sn)
                # If it's the first one, stick it in our first_type map
                if first:
                    self.first_type[type] = sn
                    first = False
                # If there was a previous node, link them if keys
                # match, or else create an implicit SN
                if prev_sn:
                    if get_message_key(prev_sn.new_class) == get_message_key(sn.old_class):
                        prev_sn.next = sn
                    else:
                        implicit_sn = ScaffoldNode(prev_sn.new_class, sn.old_class, None)
                        self.base_nodes.append(implicit_sn)
                        prev_sn.next = implicit_sn
                        implicit_sn.next = sn
                # The just-created node now becomes the previous
                prev_sn = sn

            # If there is a rename rule
            if rulechain.rename:
                # Create a scaffoldnode
                sn = ScaffoldNode(rulechain.rename.old_class, rulechain.rename.new_class, rulechain.rename)
                self.base_nodes.append(sn)

                # Same rules apply here as when we created each node
                # from chain.  Link if possible, otherwise create
                # implicit
                if first:
                    self.first_type[type] = sn
                    first = False
                if prev_sn:
                    if get_message_key(prev_sn.new_class) == get_message_key(sn.old_class):
                        prev_sn.next = sn
                    else:
                        implicit_sn = ScaffoldNode(prev_sn.new_class, sn.old_class, None)
                        self.base_nodes.append(implicit_sn)
                        prev_sn.next = implicit_sn
                        implicit_sn.next = sn                        
                prev_sn = sn
                terminal_nodes.append(sn)
            # If there was not a rename rule, this must be a terminal node
            else:
                if prev_sn:
                    terminal_nodes.append(prev_sn)
        
        # Between our partial scaffold and name remapping list, we can
        # now GENERATE rules, though we cannot yet populate the
        # subrules.

        for sn in terminal_nodes:
            key = get_message_key(sn.new_class)

            renamed = (sn.old_class._type != sn.new_class._type)

            sys_class = roslib.message.get_message_class(sn.new_class._type)

            # If we map directly to a system-defined class we're done
            if sys_class:
                new_rule = self.make_update_rule(sn.new_class, sys_class)
                R = new_rule(self, 'GENERATED.' + new_rule.__name__)
                if R.valid:
                    sn.next = ScaffoldNode(sn.new_class, sys_class, R)
                    self.base_nodes.append(sn.next)

            if renamed:
                tmp_sns = self.scaffold_range(sn.new_class._type, sn.new_class._type)

                # If we don't map to a scaffold range, we appear to be done
                if tmp_sns == []:
                    if sys_class is not None:
                        sn.next = ScaffoldNode(sn.new_class, sys_class, None)
                        self.base_nodes.append(sn.next)
                        continue

                # Otherwise look for trivial bridges
                for tmp_sn in reversed(tmp_sns):
                    tmp_key = get_message_key(tmp_sn.old_class)
                    if (key == tmp_key):
                        sn.next = tmp_sn
                        break

                # If we did not find a trivial bridge, we instead need
                # to create the right implicit rule ourselves.  This
                # is based on the ability to create a valid implicit
                # rule as LATE in the chain as possible.  We do this
                # to avoid extra conversions in some boundary
                # circumstances.
                if (sn.next is None):
                    for tmp_sn in reversed(tmp_sns):
                        new_rule = self.make_update_rule(sn.new_class, tmp_sn.old_class)
                        R = new_rule(self, 'GENERATED.' + new_rule.__name__)
                        if R.valid:
                            sn.next = ScaffoldNode(sn.new_class, tmp_sn.old_class, R)
                            self.base_nodes.append(sn.next)
                            break

            
            # If we have still failed we need to create a placeholder.  
            if (sn.next is None):
                if sys_class:
                    new_rule = self.make_update_rule(sn.new_class, sys_class)
                else:
                    new_rule = self.make_old_half_rule(sn.new_class)
                R = new_rule(self, 'GENERATED.' + new_rule.__name__)
                sn.next = ScaffoldNode(sn.new_class, None, R)
                self.base_nodes.append(sn.next)
                    

        # Now that our scaffolding is actually complete, we iterate
        # through all of our rules and generate the rules for which we
        # have scaffoldnodes, but no rule yet
        for sn in self.base_nodes:
            if (sn.rule is None):
                new_rule = self.make_update_rule(sn.old_class, sn.new_class)
                sn.rule = new_rule(self, 'GENERATED.' + new_rule.__name__)

        # Finally, we go through and try to find sub_paths for every
        # rule in the system so far
        for sn in self.base_nodes:
            sn.rule.find_sub_paths()

        # Construction should be done, we can now use the system in
        # the event that we don't have invalid update rules.


    # Add an update rule to our set of rule chains
    def add_update_rule(self, r):
        if r.valid == False:
            print >> sys.stderr, "ERROR: Update rule [%s] has valid set to False."%(r.location)
            self.false_rule_loaded = True
            return

        rulechain = self.rulechains[r.old_type]

        if r.rename_rule:
            if (rulechain.rename != None):
                print >> sys.stderr, "WARNING: Update rules [%s] and [%s] both attempting to rename type [%s]. Ignoring [%s]"%(
                    rulechain.rename.location, r.location, r.old_type, r.location)
                return

            # Search forward to make sure we havn't created a cycle
            cycle = []
            tmp = r
            while tmp:
                cycle.append(tmp)
                if (tmp.new_type == r.old_type):
                    print >> sys.stderr, "WARNING: Update rules %s introduce a renaming cycle. Ignoring [%s]"%(
                        [x.location for x in cycle],r.location)
                    return
                if (self.rulechains.has_key(tmp.new_type)):
                    tmp = self.rulechains[tmp.new_type].rename
                else:
                    break


            if rulechain.chain and (r.order <= rulechain.chain[-1].order):
                print >> sys.stderr, "WARNING: Update rule [%s] which performs rename does not have largest order number. Ignoring"%(
                    r.location)
                return

            rulechain.rename = r

        else:
            if r.order in rulechain.order_keys:
                otherind = [x.order for x in rulechain.chain].index(r.order)
                print >> sys.stderr, "WARNING: Update rules [%s] and [%s] for type [%s] have the same order number. Ignoring [%s]"%(
                    rulechain.chain[otherind].location, r.location, r.old_type, r.location)
                return
            else:
                if rulechain.rename and (r.order >= rulechain.chain[-1]):
                    print >> sys.stderr, "WARNING: Update rule [%s] has order number larger than rename rule [%s]. Ignoring"%(
                        r.location, rulechain.rename.location)
                    return
                # Insert the rule into a rule chain
                rulechain.order_keys.add(r.order)
                rulechain.chain.append(r)
                rulechain.chain.sort(key=lambda x: x.order)
                
    # Helper function to determine if all rules are valid
    def all_rules_valid(self):
        base_valid  = not False in [sn.rule.valid for sn in self.base_nodes]
        extra_valid = not False in [sn.rule.valid for sn in self.extra_nodes]
        return base_valid and extra_valid

    # Helper function to print out the definitions for all invalid rules (which include definitions)
    def get_invalid_rules(self):
        invalid_rules = []
        invalid_rule_cache = []
        for sn in self.base_nodes:
            if not sn.rule.valid:
                path_key = get_path_key(sn.old_class, sn.new_class)
                if (path_key not in invalid_rule_cache):
                    invalid_rules.append(sn.rule)
                    invalid_rule_cache.append(path_key)
        for sn in self.extra_nodes:
            if not sn.rule.valid:
                path_key = get_path_key(sn.old_class, sn.new_class)
                if (path_key not in invalid_rule_cache):
                    invalid_rules.append(sn.rule)
                    invalid_rule_cache.append(path_key)
        return invalid_rules

    # Helper function to remove non-unique rules
    def filter_rules_unique(self, rules):
        rule_cache = []
        new_rules = []
        for r in rules:
            path_key = get_path_key(r.old_class, r.new_class)
            if (path_key not in rule_cache):
                new_rules.append(r)
        return new_rules

    # Helper function to expand a list of rules to include subrules
    def expand_rules(self, rules):
        filtered = self.filter_rules_unique(rules)
        expanded = []
        for r in filtered:
            expanded.append(r)
            #print "For rule %s --> %s"%(r.old_class._type, r.new_class._type)
            expanded.extend(self.expand_rules(r.sub_rules))
        filtered = self.filter_rules_unique(expanded)
        return filtered

    def scaffold_range(self, old_type, new_type):
        try:
            tmp_sn = self.first_type[old_type]
            
            sn_range = [tmp_sn]

            found_new_type = False

            while (tmp_sn.next is not None and tmp_sn.next.new_class is not None):
#                print sn_range
                tmp_sn = tmp_sn.next
                sn_range.append(tmp_sn)
                if (tmp_sn.new_class._type == new_type):
                    found_new_type == True
                if (found_new_type and tmp_sn.new_class._type != new_type):
                    break

            return sn_range

        except KeyError:
            return []


    def find_target(self, old_class):
        key = get_message_key(old_class)

        last_class = old_class

        try:
            return self.found_targets[key]
        except KeyError:

            sys_class = roslib.message.get_message_class(old_class._type)

            if sys_class is not None:
                self.found_targets[key] = sys_class
                return sys_class

            try:
                tmp_sn = self.first_type[old_class._type]

                if tmp_sn.new_class is not None:
                    last_class = tmp_sn.new_class

                while tmp_sn.next is not None:
                    tmp_sn = tmp_sn.next

                if tmp_sn.new_class is not None:
                    last_class = tmp_sn.new_class
                    sys_class = roslib.message.get_message_class(tmp_sn.new_class._type)
                else:
                    sys_class = None

                if sys_class is not None:
                    self.found_targets[key] = sys_class
                    return sys_class
            except KeyError:
                pass

        self.found_targets[key] = None

        (pkg, msg) = last_class._type.split('/')
        try:
            pkg_dir = roslib.packages.get_pkg_dir(pkg)
        except roslib.packages.InvalidROSPkgException:
            return None
        mtypes = roslib.msgs.list_msg_types(pkg, False)
        if msg in mtypes:
            if not os.path.isfile(os.path.join(pkg_dir, os.path.join('src', pkg, 'msg', '_%s.py'%msg))):
                print >> sys.stderr, "WARNING: Package \'%s\' contains message '%s' but is not built."%(pkg,msg)
        return None
            
    # This function determines the set of rules which must be created
    # to get from the old type to the new type.
    def find_path(self, old_class, new_class):
        key = get_path_key(old_class, new_class)

        # Return any path already found in the cache
        try:
            return self.found_paths[key]
        except KeyError:
            pass

        # If the new_class is none, e.g., a message has been moved and
        # we are lacking a proper rename rule, such that find-target
        # failed, the best we can do is create a half-rule from the
        # end-point
        if new_class is None:
            sn_range = self.scaffold_range(old_class._type, "")

            found_start = False

            for (ind, tmp_sn) in reversed(zip(range(len(sn_range)), sn_range)):
                # Skip until we find the class we're trying to match
                if (tmp_sn.old_class._type != old_class._type):
                    continue
                if get_message_key(tmp_sn.old_class) == get_message_key(old_class):
                    sn_range = sn_range[ind:]
                    found_start = True
                    break

            # Next see if we can create a valid rule
            if not found_start:
                for (ind, tmp_sn) in reversed(zip(range(len(sn_range)), sn_range)):
                    if (tmp_sn.old_class._type != old_class._type):
                        continue
                    new_rule = self.make_update_rule(old_class, tmp_sn.old_class)
                    R = new_rule(self, 'GENERATED.' + new_rule.__name__)
                    if R.valid:
                        R.find_sub_paths()
                        sn = ScaffoldNode(old_class, tmp_sn.old_class, R)
                        self.extra_nodes.append(sn)
                        sn_range = sn_range[ind:]
                        sn_range.insert(0,sn)
                        found_start = True
                        break

            if sn_range == []:
                tmp_class = old_class
            else:
                tmp_class = sn_range[-1].new_class

            new_rule = self.make_old_half_rule(tmp_class)
            R = new_rule(self, 'GENERATED.' + new_rule.__name__)
            sn = ScaffoldNode(tmp_class, None, R)
            sn_range.append(sn)
            self.extra_nodes.append(sn)
            self.found_paths[key] = sn_range
            return sn_range

        # If the messages are the same, there is no actually path
        if (old_class._type == new_class._type and old_class._full_text.strip() == new_class._full_text.strip()):
            self.found_paths[key] = []
            return []

        sn_range = self.scaffold_range(old_class._type, new_class._type)

        # If we have no scaffolding, we just try to create the one path
        if sn_range == []:
            new_rule = self.make_update_rule(old_class, new_class)
            R = new_rule(self, 'GENERATED.' + new_rule.__name__)
            R.find_sub_paths()
            sn = ScaffoldNode(old_class, new_class, R)
            self.extra_nodes.append(sn)
            self.found_paths[key] = [sn]
            return [sn]


        # Search for the stop point in the scaffold
        found_stop = False

        # First look for a trivial match
        for (ind, tmp_sn) in reversed(zip(range(len(sn_range)), sn_range)):
            # Stop looking early if the classes don't match
            if (tmp_sn.new_class._type != new_class._type):
                break
            if get_message_key(tmp_sn.new_class) == get_message_key(new_class):
                sn_range = sn_range[:ind+1]
                found_stop = True
                break

        # Next see if we can create a valid rule
        if not found_stop:
            for (ind, tmp_sn) in reversed(zip(range(len(sn_range)), sn_range)):
                if (tmp_sn.new_class._type != new_class._type):
                    break
                new_rule = self.make_update_rule(tmp_sn.new_class, new_class)
                R = new_rule(self, 'GENERATED.' + new_rule.__name__)
                if R.valid:
                    R.find_sub_paths()
                    sn = ScaffoldNode(tmp_sn.new_class, new_class, R)
                    self.extra_nodes.append(sn)
                    sn_range = sn_range[:ind+1]
                    sn_range.append(sn)
                    found_stop = True
                    break

        # If there were no valid implicit rules, we suggest a new one from to the end
        if not found_stop:
            new_rule = self.make_update_rule(sn_range[-1].new_class, new_class)
            R = new_rule(self, 'GENERATED.' + new_rule.__name__)
            R.find_sub_paths()
            sn = ScaffoldNode(sn_range[-1].new_class, new_class, R)
            self.extra_nodes.append(sn)
            sn_range.append(sn)

        # Search for the start point in the scaffold
        found_start = False

        # First look for a trivial match
        for (ind, tmp_sn) in reversed(zip(range(len(sn_range)), sn_range)):
            # Skip until we find the class we're trying to match
            if (tmp_sn.old_class._type != old_class._type):
                continue
            if get_message_key(tmp_sn.old_class) == get_message_key(old_class):
                sn_range = sn_range[ind:]
                found_start = True
                break

        # Next see if we can create a valid rule
        if not found_start:
            for (ind, tmp_sn) in reversed(zip(range(len(sn_range)), sn_range)):
                if (tmp_sn.old_class._type != old_class._type):
                    continue
                new_rule = self.make_update_rule(old_class, tmp_sn.old_class)
                R = new_rule(self, 'GENERATED.' + new_rule.__name__)
                if R.valid:
                    R.find_sub_paths()
                    sn = ScaffoldNode(old_class, tmp_sn.old_class, R)
                    self.extra_nodes.append(sn)
                    sn_range = sn_range[ind:]
                    sn_range.insert(0,sn)
                    found_start = True
                    break

        self.found_paths[key] = sn_range
        return sn_range

    def migrate(self, msg_from, msg_to):
        path = self.find_path(msg_from.__class__, msg_to.__class__)

        if False in [sn.rule.valid for sn in path]:
            raise BagMigrationException("Migrate called, but no valid migration path from [%s] to [%s]"%(msg_from._type, msg_to._type))

        # Short cut to speed up case of matching md5sum:
        if path == [] or msg_from._md5sum == msg_to._md5sum:
            buff = StringIO()
            msg_from.serialize(buff)
            msg_to.deserialize(buff.getvalue())
            return

        if len(path) > 0:
            buff = StringIO()
            msg_from.serialize(buff)

            tmp_msg = path[0].old_class()
        
            tmp_msg.deserialize(buff.getvalue())

            for sn in path:
                tmp_msg = sn.rule.apply(tmp_msg)
        else:
            tmp_msg = msg_from

        buff = StringIO()
        tmp_msg.serialize(buff)
        msg_to.deserialize(buff.getvalue())

    def migrate_array(self, msg_from_array, msg_to_array):
        if len(msg_from_array) != len(msg_to_array):
            raise BagMigrationException("Migrate array called on on arrays of unequal length.")

        if len(msg_from_array) == 0:
            return

        path = self.find_path(msg_from_array[0].__class__, msg_to_array[0].__class__)

        if path is None:
            raise BagMigrationException("Migrate called, but no migration path from [%s] to [%s]"%(msg_from._type, msg_to._type))

        # Short cut to speed up case of matching md5sum:
        if path == []:
            for i in xrange(len(msg_from_array)):
                buff = StringIO()
                msg_from_array[i].serialize(buff)
                msg_to_array[i].deserialize(buff.getvalue())
            return

        for i in xrange(len(msg_from_array)):
            buff = StringIO()
            tmp_msg = path[0].old_class()
            msg_from_array[i].serialize(buff)
            tmp_msg.deserialize(buff.getvalue())
            for sn in path:
                tmp_msg = sn.rule.apply(tmp_msg)

            buff = StringIO()
            tmp_msg.serialize(buff)
            msg_to_array[i].deserialize(buff.getvalue())

    def make_update_rule(self, old_class, new_class):
        name = "update_%s_%s"%(old_class._type.replace("/","_"), old_class._md5sum)

        # We assemble the class as a string and then exec it to end up with a class
        # that can essentially print its own definition.
        classdef = "class %s(MessageUpdateRule):\n"%name
        classdef += "\told_type = \"%s\"\n"%old_class._type
        classdef += "\told_full_text = \"\"\"\n%s\n\"\"\"\n\n"%old_class._full_text.strip()
        classdef += "\tnew_type = \"%s\"\n"%new_class._type
        classdef += "\tnew_full_text = \"\"\"\n%s\n\"\"\"\n"%new_class._full_text.strip()
        classdef += "\n"
        classdef += "\torder = 0"
        classdef += "\n"

        validdef = "\tvalid = True\n"

        migratedefs = "\tmigrated_types = ["

        updatedef = "\tdef update(self, old_msg, new_msg):\n"

        old_consts = constants_from_def(old_class._type, old_class._full_text)
        new_consts = constants_from_def(new_class._type, new_class._full_text)

        if (not new_consts >= old_consts):
            validdef = "\tvalid = False\n"
            for c in (old_consts - new_consts):
                updatedef += "\t\t#Constant '%s' has changed\n"%(c[0],)
        
        old_slots = []
        old_slots.extend(old_class.__slots__)

        migrations_seen = []

        # Assign across primitives, self.migrate or self.migrate_array non-primitives
        for (s,t) in zip(new_class.__slots__, new_class._slot_types):
            warn_msg = None
            new_base_type, new_is_array, new_array_len = roslib.msgs.parse_type(t)
            try:
                ind = old_class.__slots__.index(s)
                old_slots.remove(s)
                old_base_type, old_is_array, old_array_len = roslib.msgs.parse_type(old_class._slot_types[ind])

                if new_is_array != old_is_array:
                    warn_msg = "Could not match array with nonarray"

                elif new_array_len != old_array_len:
                    if old_array_len is None:
                        warn_msg = "Converted from variable length array to fixed array of length %d"%(new_array_len)
                    elif new_array_len is None:
                        warn_msg = "Converted from fixed array of length %d to variable length"%(old_array_len)
                    else:
                        warn_msg = "Fixed length array converted from %d to %d"%(old_array_len,new_array_len)

                elif roslib.msgs.is_builtin(new_base_type):
                    if new_base_type != old_base_type:
                        warn_msg = "Primitive type changed"
                    else:
                        updatedef += "\t\tnew_msg.%s = old_msg.%s\n"%(s,s)

                else:
                    tmp_old_type = clean_name(old_base_type, old_class._type)
                    tmp_new_type = clean_name(new_base_type, new_class._type)

                    tmp_qualified_old_type = qualified_name(old_base_type, old_class._type)
                    tmp_qualified_new_type = qualified_name(new_base_type, new_class._type)

                    # Verify the type can theoretically be migrated
                    if (tmp_qualified_old_type == tmp_qualified_new_type) or \
                            (self.rename_map.has_key(tmp_qualified_old_type) and 
                             tmp_qualified_new_type in self.rename_map[tmp_qualified_old_type]):

                        if (tmp_old_type, tmp_new_type) not in migrations_seen:
                            migratedefs += "\n\t\t(\"%s\",\"%s\"),"%(tmp_old_type, tmp_new_type)
                            migrations_seen.append((tmp_old_type, tmp_new_type))

                        if not new_is_array:
                            updatedef += "\t\tself.migrate(old_msg.%s, new_msg.%s)\n"%(s,s)
                        else:
                            updatedef += "\t\tself.migrate_array(old_msg.%s, new_msg.%s, \"%s\")\n"%(s,s,new_base_type)
                    else:
                        warn_msg = "No migration path between [%s] and [%s]"%(tmp_old_type, tmp_new_type)
            except ValueError:
                warn_msg = "No matching field name in old message"

            if warn_msg is not None:
                validdef = "\tvalid = False\n"
                updatedef += "\t\t#%s\n"%warn_msg
                updatedef += "\t\tnew_msg.%s = %s\n"%(s,migration_default_value(t))
                
        migratedefs += "]\n"

        if old_slots:
            validdef = "\tvalid = False\n"
            for s in old_slots:
                updatedef += "\t\t#No field to match field %s from old message\n"%(s)

        classdef += migratedefs + '\n' + validdef + '\n' + updatedef

        printclassdef = classdef +  "\tdef get_class_def(self):\n\t\treturn \'\'\'%s\'\'\'\n"%classdef
        
        # This is probably a TERRIBLE idea?
        exec(printclassdef)
        return locals()[name]

    def make_old_half_rule(self, old_class):
        name = "update__%s__%s"%(old_class._type.replace("/","_"), old_class._md5sum)

        # We assemble the class as a string and then exec it to end up with a class
        # that can essentially print its own definition.
        classdef = "class %s(MessageUpdateRule):\n"%name
        classdef += "\told_type = \"%s\"\n"%old_class._type
        classdef += "\told_full_text = \"\"\"\n%s\n\"\"\"\n\n"%old_class._full_text.strip()
        classdef += "\tnew_type = \"\"\n"
        classdef += "\tnew_full_text = \"\"\"\n\n\"\"\"\n"
        classdef += "\n"
        classdef += "\torder = 0"
        classdef += "\n"
    
        validdef = "\tvalid = False\n"

        migratedefs = "\tmigrated_types = []\n"

        updatedef = "\tdef update(self, old_msg, new_msg):\n"
        updatedef += "\t\tpass\n"
        
        classdef += migratedefs + '\n' + validdef + '\n' + updatedef

        printclassdef = classdef +  "\tdef get_class_def(self):\n\t\treturn \'\'\'%s\'\'\'\n"%classdef
        
        # This is probably a TERRIBLE idea?
        exec(printclassdef)
        return locals()[name]

    def make_new_half_rule(self, new_class):
        name = "update_to_%s_%s"%(new_class._type.replace("/","_"), new_class._md5sum)

        # We assemble the class as a string and then exec it to end up with a class
        # that can essentially print its own definition.
        classdef = "class %s(MessageUpdateRule):\n"%name
        classdef += "\told_type = \"\"\n"
        classdef += "\told_full_text = \"\"\"\n\n\"\"\"\n\n"
        classdef += "\tnew_type = \"%s\"\n"%new_class._type
        classdef += "\tnew_full_text = \"\"\"\n%s\n\"\"\"\n"%new_class._full_text.strip()
        classdef += "\n"
        classdef += "\torder = 0"
        classdef += "\n"
    
        validdef = "\tvalid = False\n"

        migratedefs = "\tmigrated_types = []\n"

        updatedef = "\tdef update(self, old_msg, new_msg):\n"
        updatedef += "\t\tpass\n"
        
        classdef += migratedefs + '\n' + validdef + '\n' + updatedef

        printclassdef = classdef +  "\tdef get_class_def(self):\n\t\treturn \'\'\'%s\'\'\'\n"%classdef
        
        # This is probably a TERRIBLE idea?
        exec(printclassdef)
        return locals()[name]

def migration_default_value(field_type):
    if field_type in ['bool', 'byte', 'int8', 'int16', 'int32', 'int64',\
                          'char', 'uint8', 'uint16', 'uint32', 'uint64']:
        return '0'
    elif field_type in ['float32', 'float64']:
        return '0.'
    elif field_type == 'string':
        # strings, byte[], and uint8s are all optimized to be strings
        return "''"
    elif field_type.endswith(']'): # array type
        base_type, is_array, array_len = roslib.msgs.parse_type(field_type)
        if base_type in ['byte', 'uint8']:
            # strings, byte[], and uint8s are all optimized to be strings
            if array_len is not None:
                return "chr(0)*%s"%array_len
            else:
                return "''"
        elif array_len is None: #var-length
            return '[]'
        else: # fixed-length, fill values
            def_val = migration_default_value(base_type)
            return '[' + ','.join(itertools.repeat(def_val, array_len)) + ']'
    else:
        return "self.get_new_class('%s')()"%field_type

def constants_from_def(core_type, msg_def):
    core_pkg, core_base_type = roslib.names.package_resource_name(core_type)

    splits = msg_def.split('\n' + '=' * 80 + '\n')
    
    core_msg = splits[0]
    deps_msgs = splits[1:]

    # create MsgSpec representations of .msg text
    specs = { core_type: roslib.msgs.load_from_string(core_msg, core_pkg) }
    # - dependencies
#    for dep_msg in deps_msgs:
#        # dependencies require more handling to determine type name
#        dep_type, dep_spec = _generate_dynamic_specs(specs, dep_msg)
#        specs[dep_type] = dep_spec

    return set([(x.name, x.val, x.type) for x in specs[core_type].constants])
