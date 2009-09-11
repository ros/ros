#! /usr/bin/env python
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
"""
End-to-end tests of python flow-generation capabilities. 
"""

import sys, traceback, os, itertools, math, cStringIO
import os.path as path
import unittest

#TODO: array types

#disabling cleanup allows flow files to be examined but also invalidates repeated test runs
CLEANUP = True 

GENERATE_FLOWS = "generate_flows"
GENERATE_PY = "generate_py"
TEST_SERIALIZE_PY = "testserialize_py"

config = {
    GENERATE_FLOWS : False,
    GENERATE_PY : False,
    TEST_SERIALIZE_PY : False,
    }
def scratchDir(create=True):
    testDir = sys.path[0]
    dir = path.join(testDir, 'scratch')
    if create and not path.exists(dir):
        os.makedirs(dir)
    if path.exists(dir) and not path.isdir(dir):
        raise Exception("file [%s] is in the way, cannot write test files"%path.abspath(dir))
    return dir

#chain into directory containing flowgen_py
commonFlowsDir = path.join(path.dirname(sys.path[0]), '..', 'common_flows', 'flows', 'python')
print "common_flows dir is", commonFlowsDir
scriptDir = path.join(path.dirname(sys.path[0]), 'scripts')
print "script dir is", scriptDir
srcDir = path.join(path.dirname(sys.path[0]), 'src')
print "src dir is", srcDir
sys.path.append(scriptDir)
sys.path.append(srcDir) #for loading ros module
sys.path.append(commonFlowsDir) #for loading common_flows
sys.path.append(scratchDir(False)) #for loading generated python flow files
print "sys.path amended to", sys.path
print "scratch dir is", scratchDir()
import rospy, common_flows
import flowgen_py
import msgs

BUILTIN = ['int32',
           'uint32',
           'int64',
           'uint64',
           'float32',
           'float64',
           'char',
           'byte',
           'string',
           'time',
           'duration',           
          ]

MAXINT = 2147483647
MAXLONG = long('0x7fffffffffffffff', 16)
TESTVALS = {
    'int32': [0, 1024, MAXINT, -MAXINT],
    'uint32': [0, 1024, MAXINT, MAXINT*2+1],
    'int64': [0, MAXINT, MAXINT*2+1, -MAXINT, -(MAXINT*2+1)], #, MAXLONG, -MAXLONG],
    'uint64': [0, MAXINT, MAXINT*2+1, MAXLONG*2+1],
    'float32': [0.0, math.pi, 3.4028235e+38, -3.4028235e+38],
    'float64': [0.0, math.pi, 1.7976931348623157e+308, -1.7976931348623157e+308],
    'char': [chr(0), chr(127), chr(255), '\n'], #unsigned
    'byte': [0, 63, -63, 127, -127], #signed
    'string': ['', '\0', 'x', 'foobar', 'l;kjaslkjasdofiu23497xcvljjasdfl;jqeropiu1243098asdf;ljasdvl;kzxcvlkjasdf098234-098asdfjkljzxc;vas[d9fu-097234iaskldfjl;kasdjf9023409asdilfjlz;kxcjv098asd0-9f81j42l;kjads;flj0-0a-098"""l;kjasdfop9u09acvklnasdf;lkj09;ijlkjas90dfaksdjfl;kjcxvl;j\n\n\n\n\n\\)(*@$#^@#)', ''.join([chr(n) for n in xrange(0, 255)])],
    #TODO:
    'time': [],
    'duration': []    
    }

HEADER = "#auto-generate flow file for testing, okay to delete\n"

def fieldName(i):
    if i<0:
        raise Exception("i must be positive")
    if i<26:
        return chr(ord('a')+i)
    else:
        return fieldName(i%26)*(i//26 + 1)

#NOTE: its possible for multiple specs to map to the same name, in particular
#embeds of embeds, so care should be taken when writing unit tests 
def generateMsgName(spec):
    n = "test"+'_'.join([str(s) for s in spec])
    return n.replace(msgs.FLOWSEPARATOR, '_') #make filesystem legal
    
def generateMsgPath(spec):
    return path.join(scratchDir(), generateMsgName(spec)+msgs.EXT)

#TODO: rewrite so that it uses higher-level flowgen_py workings
def generatePythonFlows(files, stdout):
    outdir = scratchDir()
    flowFiles = []
    for f in files:
        (flowName, outfile) = flowgen_py.generateMsg(f, outdir, stdout)
        flowFiles.append(outfile)
    return flowFiles

def cleanup(files):
    if not CLEANUP:
        return
    for f in files:
        #print "Cleaning up", f
        try:
            if os.path.exists(f):
                os.remove(f)
        except:
            traceback.print_exc()

def generateMsgDef(types):
    return ''.join(["%s %s\n"%(t, fieldName(k)) for (k, t) in enumerate(types)])

def combos(*lists):
    if len(lists) == 0:
        yield []
    elif len(lists) == 1:
        for v in lists[0]:
            yield [v]
    else:
        for val in lists[-1]:
            cut = lists[:-1]
            for c in combos(*cut):
                copy = c[:]
                copy.append(val)
                yield copy
    
class SerializeGeneratorTestCase(unittest.TestCase):

    def simpleSerializeTest(self, types, pyfiles):
        failures = []
        for (type, pyfile) in itertools.izip(types, pyfiles):
            name = path.basename(pyfile)
            assert name
            assert name.endswith('.py'), "File [%s] does not have .py extension"%name
            name = name[:-3]
            try:
                #basic class hierarchy and importing validation
                exec "from %s import %s, Flow%s"%(name, name, name)
                serializer = eval("%s()"%name)
                flow = eval("Flow%s('foo.bar')"%name)                    
                assert isinstance(serializer, object) #__slots__ check
                assert isinstance(flow, rospy.FlowBase)

                #generate initialization values based on builtin types
                vals = combos(*[self.testVals[t] for t in type])
                #print "Testing serialization for", name
                self.testVals[name] = vals
                for val in vals:
                    #create new serializer and serialize our test data
                    serializer = eval("%s(*val)"%name)
                    try:
                        buff = cStringIO.StringIO()
                        serializer.serialize(buff)
                        serialized = buff.getvalue()
                        #test: deserialize(serialize(data)) = data
                        serializer.deserialize(serialized)
                        dvals = [getattr(serializer, f) for f in serializer.__slots__]
                    except:
                        traceback.print_exc()
                        print "FAILURE[%s]: data was %s", (name, val)
                        failures.append(name)
            except ImportError, e:
                print "Import failure", e
                failures.append(name)
        if failures:
            raise Exception("tests failed: %s"%(' '.join(failures)))
        
    def simpleProto(self, fileGen):
        """template for most builtin-type-based unit tests, which allows for testing
        of any type that doesn't include an array type. simple tests
        call with a .flow generator. All .flow files are generated and then
        the appropriate flow files are generated and tested."""
        files = []
        specs = []
        stdout = sys.stdout
        try:
            for (p, spec) in itertools.chain(*map(fileGen, BUILTIN)):
                files.append(p)
                specs.append(spec)

            #1. Generate .msg files
            if config[GENERATE_FLOWS]:
                for (p, spec) in itertools.izip(files, specs):
                    #print "Writing flow definition %s"%p
                    f = open(p, 'w')
                    try: 
                        f.write(HEADER)n
                        f.write(generateMsgDef(spec))
                    finally:
                        f.close()
                    
            #2. Generate .py files
            if config[GENERATE_PY]:
                pyfiles = generatePythonFlows(files, stdout)
                pycfiles = [(n + 'c') for n in pyfiles]

            #3. Run python serialization/deserialization tests
            if config[TEST_SERIALIZE_PY]:
                pyfiles = [f[:-4]+'py' for f in files]                
                self.simpleSerializeTest(specs, pyfiles)
        finally:
            cleanup(files)
            
    def testBuiltinSingleField(self):
        def simple(type):
            spec = (type,)
            p = generateMsgPath(spec)
            if not path.exists(p):
                yield p, spec
        self.simpleProto(simple)

    def testBuiltinMultiField(self): #this test overlaps with the combi tests
        def simpleMulti(type):
            for i in xrange(2, 6): #generate all combinations up to 5 fields
                spec = tuple(itertools.repeat(type, i))                
                p = generateMsgPath(spec)
                if not path.exists(p):
                    yield p, spec
        self.simpleProto(simpleMulti)

    def testBuiltinCombination2(self):
        def simpleCombi2(type):
            for type2 in BUILTIN: #every combination of type with one other type
                spec = (type, type2)
                p = generateMsgPath(spec)
                if not path.exists(p):
                    yield p, spec
        self.simpleProto(simpleCombi2)

    def testBuiltinCombination3(self):
        def simpleCombi3(type):
            for spec in combos(BUILTIN, BUILTIN, BUILTIN): #every combination of three
                p = path.join(scratchDir(), "test%s_%s_%s.flow"%tuple(spec))
                if not path.exists(p):
                    yield p, spec
        self.simpleProto(simpleCombi3)

    def testSimpleCommonFlowsCombination2(self):
        def simpleCommonFlowsCombi2(type):
            #every combination of common_flow with specified type
            for cf in SIMPLE_COMMON_FLOWS:
                spec = (cf, type)
                p = generateMsgPath(spec)
                if not path.exists(p):
                    yield p, spec
            for cf in SIMPLE_COMMON_FLOWS:
                spec = (type, cf) #reverse order
                p = generateMsgPath(spec)
                if not path.exists(p):
                    yield p, spec
        msgs.loadPackageDependencies('rospy', None) #this will load common_flows
        self.simpleProto(simpleCommonFlowsCombi2)

    def testSimpleCommonFlowsCombination3(self):
        def simpleCommonFlowsCombi3(type):
            #2 common_flows + a simple type
            for combo in combos(SIMPLE_COMMON_FLOWS, SIMPLE_COMMON_FLOWS):
                spec = (type, combo[0], combo[1])
                p = generateMsgPath(spec)
                if not path.exists(p):
                    yield p, spec
                spec = (combo[0], type, combo[1])
                p = generateMsgPath(spec)
                if not path.exists(p):
                    yield p, spec
                spec = (combo[0], combo[1], type)
                p = generateMsgPath(spec)
                if not path.exists(p):
                    yield p, spec
        msgs.loadPackageDependencies('rospy', None) #this will load common_flows
        self.simpleProto(simpleCommonFlowsCombi3)

    def testEmbedEmbed(self):
        """testEmbedEmbed: test multiple layers of embedding flow types within flow types"""
        def embedEmbed(type):
            spec = (type, type, 'common_flows/Pose2DFloat32')
            p = generateMsgPath(spec)
            if not path.exists(p):
                yield p, spec
            #embed level 1
            embedded = generateMsgName(spec)
            spec = (embedded, embedded, type)
            p = generateMsgPath(spec)
            if not path.exists(p):
                yield p, spec
            #embed level 2
            embedded = generateMsgName(spec)
            spec = (type, embedded, embedded)
            p = generateMsgPath(spec)
            if not path.exists(p):
                yield p, spec
        msgs.loadPackageDependencies('rospy', None) #this will load common_flows
        self.simpleProto(embedEmbed)

    
    def setUp(self):
        self.testVals = TESTVALS.copy()
        msgs.clearRegistered()
        self._cleanupScratch()
        
    def _cleanupScratch(self):
        if CLEANUP:
            dir = scratchDir(False)        
            if path.exists(dir):
                #NOTE: don't delete scratchDir itself,otherwise python imports get screwy
                print "Cleaning up scratch directory", dir
                for f in os.listdir(dir):
                    os.remove(os.path.join(dir, f))
        
    def tearDown(self):
        self._cleanupScratch()
        msgs.clearRegistered()        
        
USAGE = "usage: %(progname)s --"+" --".join(config.keys())+"\n"

def usage(progname):
    print USAGE%vars()
    
if __name__ == '__main__':
    import getopt
    optlist, args = getopt.getopt(sys.argv[1:], "h?", config.keys())
    for (field, val) in optlist:
        if field[:2] == '--' and config.has_key(field[2:]):
            config[field[2:]] = True
        elif field in ['-h', '-?']:
            usage(sys.argv[0])
            sys.exit(0)
    enabled = filter(lambda x: x, config.itervalues())
    if not enabled:
        #enable all
        print "Running tests with all options enabled"
        for key in config.iterkeys():
            config[key] = True

    msgs.set_verbose(True)
    suite = unittest.TestLoader().loadTestsFromTestCase(SerializeGeneratorTestCase)
    unittest.TextTestRunner(verbosity=2).run(suite)    

