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

import roslib; roslib.load_manifest('rosbag')

import os
import optparse
import signal
import subprocess
import sys
import UserDict

from bag import Bag, ROSBagException, ROSBagFormatException
from migration import MessageMigrator, fixbag2

def print_trans(old, new, indent):
    from_txt = '%s [%s]' % (old._type, old._md5sum)
    if new is not None:
        to_txt= '%s [%s]' % (new._type, new._md5sum)
    else:
        to_txt = 'Unknown'
    print '    ' * indent + ' * From: %s' % from_txt
    print '    ' * indent + '   To:   %s' % to_txt

def fix_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag fix INBAG OUTBAG [EXTRARULES1 EXTRARULES2 ...]')
    parser.add_option('-n', '--noplugins', action='store_true', dest='noplugins', help='do not load rulefiles via plugins')

    (options, args) = parser.parse_args(argv)

    if len(args) < 1:
        parser.error('You must pass input and output bag files.')
    if len(args) < 2:
        parser.error('You must pass an output bag file.')

    inbag_filename  = args[0]
    outbag_filename = args[1]
    rules           = args[2:]   

    ext = os.path.splitext(outbag_filename)[1]
    if ext == '.bmr':
        parser.error('Input file should be a bag file, not a rule file.')
    if ext != '.bag':
        parser.error('Output file must be a bag file.')

    outname = outbag_filename + '.tmp'

    if os.path.exists(outbag_filename):
        if not os.access(outbag_filename, os.W_OK):
            print >> sys.stderr, 'Don\'t have permissions to access %s' % outbag_filename
            sys.exit(1)
    else:
        try:
            file = open(outbag_filename, 'w')
            file.close()
        except IOError, e:
            print >> sys.stderr, 'Cannot open %s for writing' % outbag_filename
            sys.exit(1)

    if os.path.exists(outname):
        if not os.access(outname, os.W_OK):
            print >> sys.stderr, 'Don\'t have permissions to access %s' % outname
            sys.exit(1)
    else:
        try:
            file = open(outname, 'w')
            file.close()
        except IOError, e:
            print >> sys.stderr, 'Cannot open %s for writing' % outname
            sys.exit(1)

    if options.noplugins is None:
        options.noplugins = False

    migrator = MessageMigrator(rules, plugins=not options.noplugins)

    migrations = fixbag2(migrator, inbag_filename, outname)

    if len(migrations) == 0:
        print '%s %s' % (outname, outbag_filename)
        os.rename(outname, outbag_filename)
        print 'Bag migrated successfully.'
    else:
        print 'Bag could not be migrated.  The following migrations could not be performed:'
        for m in migrations:
            print_trans(m[0][0].old_class, m[0][-1].new_class, 0)
            
            if len(m[1]) > 0:
                print '    %d rules missing:' % len(m[1])
                for r in m[1]:
                    print_trans(r.old_class, r.new_class,1)
                    
        print 'Try running \'rosbag check\' to create the necessary rule files.'
        os.remove(outname)

def check_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag check BAG [-g RULEFILE] [EXTRARULES1 EXTRARULES2 ...]')
    parser.add_option('-g', '--genrules',  action='store',      dest='rulefile', default=None, help='generate a rulefile named RULEFILE')
    parser.add_option('-a', '--append',    action='store_true', dest='append',                 help='append to the end of an existing rulefile after loading it')
    parser.add_option('-n', '--noplugins', action='store_true', dest='noplugins',              help='do not load rulefiles via plugins')
    (options, args) = parser.parse_args(argv)

    if len(args) == 0:
        parser.error('You must specify a bag file to check.')
    if options.append and options.rulefile is None:
        parser.error('Cannot specify -a without also specifying -g.')
    if options.rulefile is not None:
        rulefile_exists = os.path.isfile(options.rulefile)
        if rulefile_exists and not options.append:
            parser.error('The file %s already exists.  Include -a if you intend to append.' % options.rulefile)
        if not rulefile_exists and options.append:
            parser.error('The file %s does not exist, and so -a is invalid.' % options.rulefile)

    cmd = ['rosrun', 'rosbag', 'checkbag.py']
    if options.rulefile:  cmd.extend(['-g', options.rulefile])
    if options.append:    cmd.extend(['-a'])
    if options.noplugins: cmd.extend(['-n'])
    cmd.extend(args)

    proc = subprocess.Popen(cmd)
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # ignore sigint since we're basically just pretending to be the subprocess now
    res = proc.wait()
    sys.exit(res)

def info_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag info BAGFILE1 [BAGFILE2 BAGFILE3 ...]', description='Summarize the contents of one or more bag files.')
    (options, args) = parser.parse_args(argv)

    if len(args) == 0:
        parser.error('You must specify at least 1 bag file.')

    for i, arg in enumerate(args):
        try:
            b = Bag(arg)
            print b
            b.close()
            
            #b = Bag(arg)
            #b._reader.dump()
            #b.close()
            
            if i < len(args) - 1:
                print '---'
            
        except ROSBagException, ex:
            print >> sys.stderr, 'ERROR reading %s: %s' % (arg, str(ex))
        except IOError, ex:
            print >> sys.stderr, 'ERROR reading %s: %s' % (arg, str(ex))

def play_cmd(argv):
    parser = optparse.OptionParser(usage="rosbag play BAGFILE1 [BAGFILE2 BAGFILE3 ...]", description="Play back the contents of one or more bag files in a time-synchronized fashion.")
    parser.add_option("-q", "--quiet",        dest="quiet",      default=False, action="store_true", help="suppress console output")
    parser.add_option("-i", "--immediate",    dest="immediate",  default=False, action="store_true", help="play back all messages without waiting")
    parser.add_option("--pause",              dest="pause",      default=False, action="store_true", help="start in paused mode")
    parser.add_option("--queue",              dest="queue",      default=0,     type='int', action="store", help="use an outgoing queue of size SIZE (defaults to %default)", metavar="SIZE")
    parser.add_option("--clock",              dest="clock",      default=False, action="store_true", help="publish the clock time")
    parser.add_option("--hz",                 dest="freq",       default=100,   type='float', action="store", help="use a frequency of HZ when publishing clock time (default: %default)", metavar="HZ")
    parser.add_option("-d", "--delay",        dest="delay",      default=0.2,   type='float', action="store", help="sleep SEC seconds after every advertise call (to allow subscribers to connect)", metavar="SEC")
    parser.add_option("-r", "--rate",         dest="rate",       default=1.0,   type='float', action="store", help="multiply the publish rate by FACTOR", metavar="FACTOR")
    parser.add_option("-s", "--start",        dest="sleep",      default=0.0,   type='float', action="store", help="start SEC seconds into the bag files", metavar="SEC")
    parser.add_option("--try-future-version", dest="try_future", default=False, action="store_true", help="still try to open a bag file, even if the version number is not known to the player")

    (options, args) = parser.parse_args(argv)

    if len(args) == 0:
        parser.error("You must specify at least 1 bag file to play back.")

    cmd = ["rosrun", "rosbag", "play"]

    if options.quiet:      cmd.extend(["-n"])
    if options.pause:      cmd.extend(["-p"])
    if options.immediate:  cmd.extend(["-a"])
    if options.try_future: cmd.extend(["-T"])

    if options.clock:
        cmd.extend(["-b", str(options.freq)])

    cmd.extend(["-q", str(options.queue)])
    cmd.extend(["-r", str(options.rate)])
    cmd.extend(["-s", str(options.delay)])
    cmd.extend(["-t", str(options.sleep)])

    cmd.extend(args)

    proc = subprocess.Popen(cmd)
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # ignore sigint since we're basically just pretending to be the subprocess now
    res = proc.wait()
    sys.exit(res)

def record_cmd(argv):
    parser = optparse.OptionParser(usage="rosbag record TOPIC1 [TOPIC2 TOPIC3 ...]",
                                   description="Record a bag file with the contents of specified topics.",
                                   formatter=optparse.IndentedHelpFormatter())

    parser.add_option("-a", "--all",           dest="all",      default=False, action="store_true",        help="record all topics")
    parser.add_option("-q", "--quiet",         dest="quiet",    default=False, action="store_true",        help="suppress console output")
    parser.add_option("-o", "--output-prefix", dest="prefix",   default=None,  action="store",             help="prepend PREFIX to beginning of bag name (name will always end with date stamp)")
    parser.add_option("-O", "--output-name",   dest="name",     default=None,  action="store",             help="record to bag with namename NAME.bag")
    parser.add_option("--split",               dest="split",    default=0,     type='int', action="store", help="split bag into files of size SIZE", metavar="SIZE")
    parser.add_option("-b", "--buffsize",      dest="buffsize", default=256,   type='int', action="store", help="use in internal buffer of SIZE MB (Default: %default, 0 = infinite)", metavar="SIZE")
    parser.add_option("-l", "--limit",         dest="num",      default=0,     type='int', action="store", help="only record NUM messages on each topic")
    #parser.add_option("-z", "--zlib",          dest="zlib",     default=False, action="store_true",        help="use ZLIB compression")
    parser.add_option("-j", "--bz2",           dest="bz2",      default=False, action="store_true",        help="use BZ2 compression")

    (options, args) = parser.parse_args(argv)

    if len(args) == 0 and not options.all:
        parser.error("You must specify a topic name or else use the '-a' option.")

    if options.prefix is not None and options.name is not None:
        parser.error("Can't set both prefix and name.")

    cmd = ['rosrun', 'rosbag', 'record']

    cmd.extend(['-m', str(options.buffsize)])
    cmd.extend(['-c', str(options.num)])
    cmd.extend(['-S', str(options.split)])

    if options.prefix: cmd.extend(["-f", options.prefix])
    if options.name:   cmd.extend(["-F", options.name])
    if options.all:    cmd.extend(["-a"])
    #if options.zlib:   cmd.extend(["-z"])
    if options.bz2:    cmd.extend(["-j"])

    cmd.extend(args)

    proc = subprocess.Popen(cmd)
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # ignore sigint since we're basically just pretending to be the subprocess now
    res = proc.wait()
    sys.exit(res)

def filter_cmd(argv):
    def expr_eval(expr):
        def eval_fn(topic, m, t):
            return eval(expr)
        return eval_fn

    parser = optparse.OptionParser(usage="""rosbag filter INBAG OUTBAG EXPRESSION

EXPRESSION can be any Python-legal expression.

The following variables are available:
 * topic: name of topic
 * m: message
 * t: time of message (t.secs, t.nsecs)""")
    parser.add_option('--print', dest='verbose_pattern', default=None, metavar='PRINT-EXPRESSION', help='Python expression to print for verbose debugging. Uses same variables as filter-expression')

    options, args = parser.parse_args(argv)
    if len(args) == 0:
        parser.error('You must specify an in bag, an out bag, and an expression.')
    if len(args) == 1:
        parser.error('You must specify an out bag and an expression.')
    if len(args) == 2:
        parser.error("You must specify an expression.")
    if len(args) > 3:
        parser.error("Too many arguments.")

    inbag_filename, outbag_filename, expr = args

    if not os.path.isfile(inbag_filename):
        print >> sys.stderr, "Cannot locate input bag file [%s]" % inbag_filename
        sys.exit(2)

    filter_fn = expr_eval(expr)

    outbag = Bag(outbag_filename, 'w')
    inbag  = Bag(inbag_filename)

    try:
        if options.verbose_pattern:
            verbose_pattern = expr_eval(options.verbose_pattern)
    
            for topic, msg, t in inbag.readMessages():
                if filter_fn(topic, msg, t):
                    print "MATCH", verbose_pattern(topic, msg, t)
                    outbag.write(topic, msg, t)
                else:
                    print "NO MATCH", verbose_pattern(topic, msg, t)          
        else:
            for topic, msg, t in inbag.readMessages():
                if filter_fn(topic, msg, t):
                    outbag.write(topic, msg, t)
    finally:
        inbag.close()
        outbag.close()

class RosbagCmds(UserDict.UserDict):
    def __init__(self):
        UserDict.UserDict.__init__(self)

        self['help'] = self.help_cmd

    def get_valid_cmds(self):
        str = "Valid commands:\n"
        for k in sorted(self.keys()):
            str += "    * %s\n" % k

        return str

    def help_cmd(self,argv):
        argv = [a for a in argv if a != '-h' and a != '--help']

        if len(argv) == 0:
            print 'Usage: rosbag <command> [options] [args]'
            print ''
            print self.get_valid_cmds()
            print 'For more information please visit the rosbag wiki page: http://code.ros.org/wiki/rosbag'
            print ''
            return

        cmd = argv[0]
        if cmd in self:
            self[cmd](['-h'])
        else:
            print >> sys.stderr, 'Invalid command: %s' % cmd
            print >> sys.stderr, ''
            print >> sys.stderr, self.get_valid_cmds()

def rosbagmain(argv=None):
    cmds = RosbagCmds()
    cmds['record'] = record_cmd
    cmds['play']   = play_cmd
    cmds['info']   = info_cmd
    cmds['check']  = check_cmd
    cmds['fix']    = fix_cmd
    cmds['filter'] = filter_cmd

    if argv is None:
        argv = sys.argv

    if '-h' in argv or '--help' in argv:
        argv = [a for a in argv if a != '-h' and a != '--help']
        argv.insert(1, 'help')

    if len(argv) > 1:
        cmd = argv[1]
    else:
        cmd = 'help'

    if cmd in cmds:
        cmds[cmd](argv[2:])
    else:
        cmds['help']([cmd])
