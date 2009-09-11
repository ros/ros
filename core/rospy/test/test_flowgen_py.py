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
Lower-level tests of functions in test_flowgen_py. Mostly
tests support functions for the actual flow generation. See
L{test_flowgen} for higher-level end-to-end tests.
"""

import sys, traceback
import os.path as path
import unittest

#chain into directory containing flowgen_py
scriptDir = path.join(path.dirname(sys.path[0]), 'scripts')
sys.path.append(scriptDir)
from flowgen_py import *

class FlowNameTestCase(unittest.TestCase):
    legalNames = ['Int32Flow', 'Foo', 'F', 'Flow32', 'a', 'a1', ]
    illegalNames = [None, 'Int.Flow', '32Flow', '2', '.', '', ]

    def testLegalFlowNames(self):
        for n in self.legalNames:
            assert isLegalFlowName(n), "Should be legal: %s"%n

    def testIllegalFlowNames(self):
        for n in self.illegalNames:
            assert not isLegalFlowName(n), "Should be illegal: %s"%n

class StructTestCase(unittest.TestCase):
    legalStructsSingle = [
        ('i', ['int32']),
        ('I', ['uint32']),
        ('q', ['int64']),
        ('Q', ['uint64']),
        ('f', ['float32']),
        ('d', ['float64']),
        ('c', ['char']),
        ('b', ['byte']),
        ]
    legalStructsMulti = [
        ('i', ['int32']),
        ('I', ['uint32']),
        ('l', ['int64']),
        ('L', ['uint64']),
        ('f', ['float32']),
        ('d', ['float64']),
        ('c', ['char']),
        ('b', ['byte']),
        ]
    illegalStructs = [
        None, [], [1], ['blah'],
        ['list[int32]'], ['map[int32]'],
        ['int32', 'list[int32]'],
        ['int32', 'blah'], ['blah', 'int32'],
        ]
    def testLegalStructsSingle(self):
        for s in self.legalStructsSingle:
            assert s[0] == getStructPattern(s[1]), "%s does not match expected, %s, for %s"%(getStructPattern(s[1]), s[0], s[1])

    def testIllegalStructs(self):
        for s in self.illegalStructs:
            assert getStructPattern(s) is None, "%s should be None"%s

class FlowSpecTestCase(unittest.TestCase):
    flatIdent = [
        FlowSpec(['int32', 'float32'], ['x', 'y']),
        FlowSpec(['int32'], ['x']),
        FlowSpec(['int32[]'], ['x']),
        FlowSpec(['int32', 'int32', 'int32', 'float64'], ['x', 'y', 'z', 't']),
        FlowSpec(['int32', 'int32', 'int32[]', 'float64[]'], ['x', 'y', 'z', 't']),
        ]
    flatNonIdent = [
        FlowSpec(['Point2D'], ['p1']),
        FlowSpec(['Point2D', 'Point2D'], ['p1', 'p2']),
        FlowSpec(['Point2D', 'Point2D[]', 'Point2D'], ['p1', 'parray', 'p2']),
        FlowSpec(['Rect'], ['r']),
        FlowSpec(['Rect', 'Rect[]'], ['r', 'rectArray']),
        ]
    flattened = [
        FlowSpec(['int32', 'int32'], ['p1.x', 'p1.y']),
        FlowSpec(['int32', 'int32', 'int32', 'int32'], ['ul.x', 'ul.y', 'lr.x', 'lr.y']),
        FlowSpec(['int32', 'int32', 'Point2D[]', 'int32', 'int32'], ['ul.x', 'ul.y', 'parray', 'lr.x', 'lr.y']),
        FlowSpec(['int32', 'int32', 'int32', 'int32'], ['ul.x', 'ul.y', 'lr.x', 'lr.y']),
        FlowSpec(['int32', 'int32', 'int32', 'int32', 'Rect[]'], ['ul.x', 'ul.y', 'lr.x', 'lr.y', 'rectArray']),
        ]
    def testEq(self):
        for ident in self.flatIdent:
            assert ident == ident
    def testNe(self):
        for (i, val) in enumerate(self.flatIdent):
            if i > 0:
                assert val != self.flatIdent[i-1]
    def testFlatten(self):
        REGISTERED_TYPES['Point2D'] = FlowSpec(['int32', 'int32'], ['x', 'y'])
        REGISTERED_TYPES['Point3D'] = FlowSpec(['int32', 'int32', 'int32'], ['x', 'y', 'z'])
        REGISTERED_TYPES['Rect'] = FlowSpec(['Point2D', 'Point2D'], ['ul', 'lr'])
        for ident in self.flatIdent:
            assert ident == ident.flatten()

    def tearDown(self):
        REGISTERED_TYPES = {}

    
class SerializeGeneratorTestCase(unittest.TestCase):
    legalSimpleDefs = [
        FlowSpec(['int32', 'float32'], ['x', 'y']),
        FlowSpec(['int32'], ['x']),
        FlowSpec(['int32', 'int32', 'int32', 'float64'], ['x', 'y', 'z', 't']),
        ]
    legalSimpleVals = [
        "buff.write(struct.pack('<if', self.x, self.y))",
        "buff.write(struct.pack('<i', self.x))",
        "buff.write(struct.pack('<iiid', self.x, self.y, self.z, self.t))",
        ]
    illegalDefs = [
        FlowSpec(['blint32'], ['x']),
        None,
        ]
    legalMixedDefs = [
        FlowSpec(['int32', 'int32[]'], ['i', 'l']), #prim, array
        FlowSpec(['int32[]', 'int32'], ['l', 'i']), #array, prim
        FlowSpec(['int32[]', 'int32', 'float32'], ['l', 'i', 'f']), #array, prim, prim
        FlowSpec(['int32', 'int32[]', 'int32[]', 'int32'], ['i1', 'l1', 'l2', 'i2']), #prim, array, array, prim
        FlowSpec(['int32', 'float32', 'int32[]'], ['i', 'f', 'l']), #prim, prim, array
        ]
    legalEmbeddedDefs = [
        FlowSpec(['Point2D'],['p1']),
        FlowSpec(['Point2D', 'Point2D'],['p1', 'p2']),
    ]
    legalEmbeddedVals = [
        "buff.write(struct.pack('<ii', self.p1.x, self.p1.y))",
        "buff.write(struct.pack('<ii', self.p1.x, self.p1.y))"+"buff.write(struct.pack('<ii', self.p2.x, self.p2.y))",
    ]
    legalArrayDefs = [
        FlowSpec(['int32[]'],['vals']),
        FlowSpec(['int32[10]'],['fixedVals']),
        FlowSpec(['Point2D[10]'],['fixedPointVals']),
    ]
    legalArrayVals = [
        'pass',
        'pass',
        'pass',
        ]
    def testSimpleTypesDict(self):
        """cross validate our SIMPLE_TYPES and SIMPLE_TYPES_DICT lookup table"""
        for s in SIMPLE_TYPES:
            assert SIMPLE_TYPES_DICT.has_key(s)
            val = SIMPLE_TYPES_DICT[s]
            assert type(val) == str and len(val) == 1
        for k in SIMPLE_TYPES_DICT.iterkeys():
            assert k in SIMPLE_TYPES
            
    def testSimple(self):
        for test, val in zip(self.legalSimpleDefs, self.legalSimpleVals):
            try:
                source = list(serializerGenerator(test))
                assert ''.join(source) == val, "%s != %s"%(''.join(source), val)
            except Exception, e:
                print "Exception %s raised for test. Val is : %s"%(e, val)
                raise
    def testIllegal(self):        
        for test in self.illegalDefs:
            try:
                list(serializerGenerator(test))
                if test:
                    self.fail("expected seralizerGenerator to raise Exception for invalid paramter: %s, %s"%(test.names, test.values))
                else:
                    self.fail("expected seralizerGenerator to raise Exception for None parameter")
            except:
                pass

    def testArray(self):
        REGISTERED_TYPES['Point2D'] = FlowSpec(['int32', 'int32'], ['x', 'y'])
        for test, val in zip(self.legalArrayDefs, self.legalArrayVals):
            try:
                source = list(serializerGenerator(test))
                #print "Array", source
                #TODO: values
                #assert ''.join(source) == val, "%s != %s"%(''.join(source), val)
            except Exception, e:
                #traceback.print_exc()
                print "Exception %s raised for test. Val is : %s"%(e, val)
                raise


    def testEmbedded(self):
        REGISTERED_TYPES['Point2D'] = FlowSpec(['int32', 'int32'], ['x', 'y'])
        for test, val in zip(self.legalEmbeddedDefs, self.legalEmbeddedVals):
            try:
                source = list(serializerGenerator(test))
                assert ''.join(source) == val, "%s != %s"%(''.join(source), val)
            except Exception, e:
                print "Exception %s raised for test. Val is : %s"%(e, val)
                raise

        
    def tearDown(self):
        REGISTERED_TYPES = {}

if __name__ == '__main__':
    unittest.main()
    
