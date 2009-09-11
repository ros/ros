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

import os
import sys
import inspect
from cStringIO import StringIO
import string
import collections

import roslib.scriptutil

# Anything outside the scope of these primitives is a submessage
_PRIMITIVES = ['byte','int8','int16','int32','int64','char','uint8','uint16','uint32','uint64','float32','float64','string','time']



class BagMigrationException(Exception):
    pass



# Helper function to strip out roslib and package name from name usages.
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



# Class to use for defining a rule which updates a message explicitly or implicitly
class MessageUpdateRule(object):
    type = ""
    new_type = None
    old_full_text = ""
    new_full_text = ""
    migrated_types = []

    valid = False

    def __init__(self, migrator, location):
        # Every rule needs to hang onto the migrator so we can potentially use it
        self.migrator = migrator
        self.location = location

        if new_type == None:
            self.new_type = type

        # Instantiate types dynamically based on definition
        self.old_types = roslib.genpy.generate_dynamic(self.type, self.old_full_text)
        self.new_types = roslib.genpy.generate_dynamic(self.new_type, self.new_full_text)

        # Make the old/new classes convenient
        self.old_class = self.old_types[self.type]
        self.new_class = self.new_types[self.new_type]

        # We have not populated our implicit rules (and ideally should
        # wait until all explicit rules have been loaded before doing
        # this)
        self.implicit_rules_done = False
        self.sub_valid = False

    def populate_implicit_rules(self, warn=False):
        # For any migrated type the user might want to use, we must
        # make sure the migrator had found a path for it It would be
        # nice not to need these through performing some kind of other
        # inspection of the update rule.
        self.sub_valid = True
        for (t1,t2) in self.migrated_types:
            try:
                tmp_old_class = self.get_old_class(t1)
            except KeyError:
                print >> sys.stderr, "WARNING: Within rule [%s], specified migrated type [%s] not found in old message types"%(self.location,t1)
                self.sub_valid = False
                continue
            try:
                tmp_new_class = self.get_new_class(t2)
            except KeyError:
                print >> sys.stderr, "WARNING: Within rule [%s], specified migrated type [%s] not found in new message types"%(self.location,t2)
                self.sub_valid = False
                continue
            if (tmp_new_class._md5sum != tmp_old_class._md5sum):
                if not self.migrator.find_path(tmp_old_class, tmp_new_class):
                    if warn:
                        print >> sys.stderr, "WARNING: Within rule [%s] cannot migrate from subtype [%s] to [%s].."%(
                            self.location, t1, t2)
                        self.sub_valid = False
                        continue
        self.implicit_rules_done = True

    def get_new_class(self,t):
        try:
            try:
                return self.new_types[t]
            except KeyError:                
                return self.new_types['roslib/' + t]
        except KeyError:
            return self.new_types[self.new_type.split('/')[0] + '/' + t]

    def get_old_class(self,t):
        try:
            try:
                return self.old_types[t]
            except KeyError:                
                return self.old_types['roslib/' + t]
        except KeyError:
            return self.old_types[self.old_type.split('/')[0] + '/' + t]


    def migrate(self, msg_from, msg_to):
        tmp_msg_from = clean_name(msg_from._type, self.new_type)
        tmp_msg_to = clean_name(msg_to._type, self.type)
        if (tmp_msg_from, tmp_msg_to) not in self.migrated_types:
            raise BagMigrationException("Rule [%s] tried to perform a migration from old [%s] to new [%s] not listed in migrated_types"%(self.location, tmp_msg_from, tmp_msg_to))
        self.migrator.migrate(msg_from, msg_to)

    def migrate_array(self, msg_from_array, msg_to_array, msg_to_class):
        while len(msg_to_array) > 0:
            msg_to_array.pop()

        msg_to_array.extend( [msg_to_class() for i in xrange(len(msg_from_array))] )
        for i in xrange(len(msg_from_array)):
            self.migrate(msg_from_array[i], msg_to_array[i])

    def print_def(self):
        pass

    def apply(self, old_msg_in, raw_in=False, raw_out=False):
        if not self.valid:
            raise BagMigrationException("Attempted to apply an invalid rule")
        if not self.implicit_rules_done:
            raise BagMigrationException("Attempted to apply a rule without building up its implicit rules")
        if not self.sub_valid:
            raise BagMigrationException("Attempted to apply a rule without valid sub-rules")

        # If raw_in we must deserialize
        if raw_in:
            old_msg = self.old_class()
            old_msg.deserialize(old_msg_in[0])
        else:
            old_msg = old_msg_in

        # Apply update rule
        new_msg = self.new_class()
        self.update(old_msg, new_msg)

        # If raw_out we must serialize
        if raw_out:
            buff = StringIO()
            new_msg.serialize(buff)
            return (self.new_class, data)
        else:
            return new_msg
            
    def update(self, old_msg, new_msg):
        raise BagMigrationException("Tried to use rule without update overidden")



# Class to use for representing that a type has changed to a new name.
# This class doesn't actually get applied but is rather indirectly used
# to create a MessageUpdateRule
class MessageRenameRule(MessageRule):
    old_type = ""
    new_type = ""
    new_full_text = ""

    def __init__(self, location):
        # Every rule needs to hang onto the migrator so we can potentially use it
        self.location = location

        # Instantiate types dynamically based on definition
        self.new_types = roslib.genpy.generate_dynamic(self.new_type, self.full_text)
        self.new_class = self.new_types[self.type]



# A class for book-keeping about rule-chains
class RuleChain(object):
    def __init__(self):
        self.chains = []
        self.order_keys 
        self.rename = None



class MessageMigrator(object):
    def __init__(self, input_rule_modules=[]):

        # We use the rulechains to scaffold our initial creation of
        # implicit rules.  Each RuleChain is keyed off of a type and
        # consists of an ordered set of update rules followed by an
        # optional rename rule.  For the system rule definitions to be
        # valid, all members of a rulechains must be connectable via
        # implicit rules and all rulechains must terminate in a known
        # system type which is also reachable by an implicit rule.
        self.rulechains = collections.defaultdict(RuleChain)
        
        # As we load or create rules they are stored in our rule
        # dictionary and keyed by the tuple (type, md5sum).  Since
        # paths through the system may only be convergent (renames
        # must be unique, cycles are disallowed, and updates are
        # ordered), if a path is known to exist, it can be followed
        # trivially by repeated application of unique rules for the
        # correct (type, md5sum) pair.
        self.rules = {}

        self.missing_names = {}

        rule_modules = []
        
        # To make debugging easy we can pass in a list of local
        # rulefiles, which are technically python modules on our path.
        # Since bash does tabcompletion on filenames, we strip of
        # ".py" which probably gives the INVALID assumption that this
        # will work for an arbitrary path.
        for r in input_rule_modules:
            if (r[-3:] == ".py"):
                r = r[:-3]
            try:
                rule_modules.append((__import__(r), r))
            except ImportError:
                print >> sys.stderr, "Cannot load rule module [%s] in local package [%s]"%r

        # Alternatively the preferred method is to load definitions
        # from the rosbagmigration ruleset export flag.
        for pkg in roslib.scriptutil.rospack_depends_on_1('rosbagmigration')
            m_file = roslib.manifest.manifest_file(pkg, True)
            m = roslib.manifest.parse_file(m_file)
            p_rules = m.get_export('rosbagmigration', 'ruleset')
            roslib.load_manifest(pkg)
            for r in p_rules:
                try:
                    mod = __import__(r)

                    for sub_mod in r.split('.')[1:]:
                        mod = getattr(mod, sub_mod)

                    rule_modules.append((mod, r))
                except ImportError:
                    print >> sys.stderr, "Cannot load rule file [%s] in package [%s]"%(r, pkg)

        # For each module we've loaded, we find all instances of Rename and Update rules and add them.
        for (mod, location_base) in rule_modules:
            for m in dir(mod):
                m_attr = getattr(mod, m)
                if inspect.isclass(m_attr):
                    if (not m_attr == MessageRenameRule) and issubclass(m_attr, MessageRenameRule):
                        self.add_rename_rule(m_attr(self, location + '.' + m_attr.__name__))
                    elif (not m_attr == MessageUpdateRule) and issubclass(m_attr, MessageUpdateRule):
                        self.add_rename_rule(m_attr(self, location + '.' + m_attr.__name__))

    # Add a rename rule to our set of rule chains
    def add_rename_rule(self, r):
        rulechain = self.rulechains[r.old_type]
        if (rulechain.rename != None):
            print >> sys.stderr, "WARNING: Rename rules [%s] and [%s] both attempting to rename type [%s]."%(
                rulechain.rename.location, r.location, r.type)
            return
        # Search forward to make sure we havn't created a cycle
        cycle = [r]
        tmp = r
        while tmp:
            if (self.rulechains.has_key(tmp.new_type)):
                tmp = self.rulechains[tmp.new_type].rename
                if tmp:
                    cycle.append(tmp)
                    if (tmp.new_type == r.old_type):
                        print >> sys.stderr, "WARNING: Rename rules %s introduce a renaming cycle."%([x.location for x in cycles])
                        return
        rulechain.rename = r

    # Add an update rule to our set of rule chains
    def add_update_rule(self, r):
        if r.valid == False:
            print >> sys.stderr, "WARNING: Update rule [%s] has valid set to False. Ignoring."%(r.location)
            return

        rulechain = self.rulechains[r.type]
        if r.order in rulechain.order_keys:
            otherind = [x.order for x in r.chain].index(r.order)
            print >> sys.stderr, "WARNING: Update rules [%s] and [%s] for type [%s] have the same order number. "%(
                rulechain.chain[otherind].location, r.location, r.type)
            return
        else:
            # Insert the rule into a rule chain
            rulechain.order_keys.add(r.order)
            rulechain.chain.append(r)
            rulechain.chain.sort(key=lambda x: x.order)

            # For the rare update rule that also changes typename
            # Note: when populating implicit rules the chain 
            if (r.type != r.new_type):
                R = self.make_rename_rule(r.type, r.new_class)
                ruleinst = R(self, r.location)
                self.add_rename_rule(ruleinst)

            # Insert the rule into the set of all rules
            self.rules[(r.type, r.old_class._md5sum)] = r

    # We require that we can find a path through existing explicit (and created implicit) rules
    def populate_implicit_rules(self):
        for (type,rulechain) in self.rulechains.iteritems():

            # Populate necessary implicit rules for each element in the chain
            for r in rulechain.chain:
                r.populate_implicit_rules(warn=True)

            # Populate implicit rules between each element in the chain
            for (r1, r2) in zip(rulechain.chain[1:], rulechain.chain[:-1]):
                if not find_implicit_path(r1.new_class, r2.old_class):
                    print >> sys.stderr, "WARNING: Ordered explicit rules [%s] and [%s] for type [%s] do not chain implicitly."%(
                        r1.location, r2.location, type)

            # Populate implicit rules from the end of the chain to the rename step
            if rulechain.chain and rulechain.rename:
                if not find_implicit_path(rulechain.chain[-1].new_class, rulechain.rename.class):
                    print >> sys.stderr, "WARNING: Last update rule [%s] and rename rule [%s] for type [%s] do not chain implicitly."%(
                        rulechain.chain[-1].location, rc.rename.location, type)

            if rulechain.rename:
                # Populate implicit rules from a rename to a new chain or system class
                if self.rulechains.has_key(rulechain.rename.new_type):
                    if not find_path_to_chain(rulechain.rename.class, self.rulechains[rulechain.rename.new_type]):
                        print >> sys.stderr, "WARNING: Rename rule [%s] and rulechain for type [%s] do not chain implicitly."%(
                        rulechain.rename.location, rulechain.rename.new_type)
                else:
                    if not find_path_to_system(rulechain.rename.class):
                        print >> sys.stderr, "WARNING: Rename rule [%s] and system type [%s] do not chain implicitly."%(
                            rulechain.rename.location, system_class._type)
            else:
                # Populate implicit rules from the end of the chain to a system class
                if rulechain.chain:
                    if not find_path_to_system(rulechain.chain[-1].new_class):
                        print >> sys.stderr, "WARNING: Last update rule [%s] and system type [%s] do not chain implicitly."%(
                            rulechain.rename.location, system_class._type)
                
    # Helper function to determine if all rules are valid
    def all_rules_valid(self):
        return not False in [r.valid for (k,r) in self.rules.iteritems()]

    # Helper function to print out the definitions for all invalid rules (which include definitions)
    def print_invalid_rules(self):
        for (k,r) in self.rules.iteritems():
            if not r.valid:
                r.print_def()


    def find_path_to_system(self, old_class):
        # The first fraction of code deals with finding our target class
        # this is not relevent if new_class is passed in.

        # If new_class is None, see if there is a a message in the
        # system with the right name
        if new_class is None:
            new_class = roslib.scriptutil.get_message_class(old_class._type)

        # If we still couldn't find anything, we're going to need to
        # apply a rule.  A rename rule might get this to match our
        # system eventually, hence the recursion.
        if new_class is None:
            try:
                R = self.update_rules[(old_class._type, old_class._md5sum)]

                # Keep searching from this new point
                return self.find_path(R.new_class, None)

            # Otherwise make a rename rule, which means we don't have
            # something valid to do since we can't automagically
            # determine what message in our ROS install to rename to
            except KeyError:
                new_rule = self.make_rename_rule(old_class)
                self.add_rule(new_rule(self))
                return None
        pass

    def find_path_to_chain(self, old_class, chain):
        pass

    
    # Find an implicit path that we know must be there.  This is most
    # useful for chaining together explicit rules in our rulechain.
    def find_implicit_path(self, old_class, new_class):
        if (old_class._md5sum == new_class._md5sum):
            return true
        else:
            new_rule = self.make_update_rule(old_class, new_class)
            R = new_rule(self, 'GENERATED.' + new_rule.__name__)
            R.populate_implicit_rules(warn=True)
            self.add_rule(R)
            return R.valid

    # This function determines the set of rules which must be created
    # to get from the old type to the new type.  Passing in None as
    # the new type will search through the current ROS tree for a
    # match based on type name.
    def find_path(self, old_class, new_class):

        # If the md5sum matches the existing named definition, we're
        # fine and a migration call will succeed trivially
        if (old_class._md5sum == new_class._md5sum):
            return True
        else:
            try:
                R = self.rules[(old_class._type, old_class._md5sum)]

            # Keep searching from this new point
            return self.find_path(R.new_class, new_class)
        except KeyError:
            new_rule = self.make_update_rule(old_class, new_class)
            R = new_rule(self)
            self.add_rule(R)
            return R.new_class

    def migrate(self, msg_from, msg_to):

        tmp_msg = msg_from

        # If messages haved matched md5sums, we handle it with serialization
        while (tmp_msg._md5sum != msg_to._md5sum):
            try:
                R = self.rules[(tmp_msg._type, tmp_msg._md5sum)]
            except KeyError:
                raise BagMigrationException("Could not find an update rule for [%s] [%s]"%(msg_from._type, msg_from._md5sum))
            if not R.valid:
                raise BagMigrationException("Found an invalid update rule for [%s] [%s]"%(msg_from._type, msg_from._md5sum))            
            tmp_msg = R.apply(tmp_msg)

        # Now that we have migrated, do the copy with a serialize/deserialize pair
        # There are likey more efficient ways to do this
        buff = StringIO()
        tmp_msg.serialize(buff)
        msg_to.deserialize(buff.getvalue())

    def make_rename_rule(self, old_type, new_class):
        name = "rename_%s"%old_class._md5sum

        # We assemble the class as a string and then exec it to end up with a class
        # that can essentially print its own definition.
        classdef = "class %s(MessageRenameRule):\n"%name
        classdef += "\told_type = \"%s\"\n"%old_type
        classdef += "\tnew_type = \"%s\"\n"%new_class._type
        classdef += "\tnew_full_text = \"\"\"\n%s\n\"\"\"\n"%new_class._full_text.strip()

        exec(classdef)
        return locals()[name]

    def make_update_rule(self,old_class, new_class):
        name = "update_%s"%old_class._md5sum

        # We assemble the class as a string and then exec it to end up with a class
        # that can essentially print its own definition.
        classdef = "class %s(MessageUpdateRule):\n"%name
        classdef += "\ttype = \"%s\"\n"%old_class._type
        if (old_class._type != new_class._type):
            classdef += "\tnew_type = \"%s\"\n"%new_class._type
        classdef += "\told_full_text = \"\"\"\n%s\n\"\"\"\n"%old_class._full_text.strip()
        classdef += "\tnew_full_text = \"\"\"\n%s\n\"\"\"\n"%new_class._full_text.strip()
        classdef += "\n"
    
        validdef = "\tvalid = True\n"

        migratedefs = "\tmigrated_types = [\n"

        updatedef = "\tdef update(self, old_msg, new_msg):\n"
        
        # Assign across primitives, self.migrate non-primitives
        for (s,t) in zip(new_class.__slots__, new_class._slot_types):
            warn_msg = None
            new_base_type, new_is_array, new_array_len = roslib.msgs.parse_type(t)
            try:
                ind = old_class.__slots__.index(s)
                old_base_type, old_is_array, old_array_len = roslib.msgs.parse_type(old_class._slot_types[ind])

                if (new_is_array != old_is_array):
                    warn_msg = "Could not match array with nonarray"

                elif (new_array_len != old_array_len):
                    warn_msg = "Fixed array lengths don't match"

                elif (new_base_type in _PRIMITIVES):
                    if (new_base_type != old_base_type):
                        warn_msg = "Primitive type changed"
                    else:
                        updatedef += "\t\tnew_msg.%s = old_msg.%s\n"%(s,s)

                else:
                    tmp_old_type = clean_name(old_base_type, old_class._type)
                    tmp_new_type = clean_name(new_base_type, new_class._type)

                    migratedefs += "\t\t(\"%s\",\"%s\"),"%(tmp_old_type, tmp_new_type)
                    if not new_is_array:
                        updatedef += "\t\tself.migrate(old_msg.%s, new_msg.%s)\n"%(s,s)
                    else:
                        updatedef += "\t\tself.migrate_array(old_msg.%s, new_msg.%s, self.get_new_class(%s))\n"%(s,s,t[:-2])
            except ValueError:
                warn_msg = "No matching field name in old message"

            if warn_msg is not None:
                validdef = "\tvalid = False\n"
                updatedef += "\t\t#%s\n"%warn_msg
                updatedef += "\t\tnew_msg.%s = %s\n"%(s,migration_default_value(t))

        migratedefs += "\n\t]\n"

        classdef += migratedefs + '\n' + validdef + '\n' + updatedef

        printclassdef = classdef +  "\tdef print_def(self):\n\t\tprint \'\'\'%s\'\'\'\n"%classdef
        
        # Is this a TERRIBLE idea?
        exec(printclassdef)
        return locals()[name]


def migration_default_value(field_type):
    if field_type in ['byte', 'int8', 'int16', 'int32', 'int64',\
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
        return "self.new_types[%s]"%field_type
