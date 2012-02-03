#!/usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import unittest

from rosmake import parallel_build

class TestDependencyTracker(unittest.TestCase):
    def setUp(self):
        self.deps = {}
        self.deps1 = {}
        self.deps["a"] = [ "b", "c", "d","e"]
        self.deps1["a"] = ["b"]
        self.deps["b"] = ["c"]
        self.deps1["b"] = ["c"]
        self.deps["d"] = ["c", "e"]
        self.deps1["d"] = ["c", "e"]
        self.dt = parallel_build.DependencyTracker()
        self.dt.load_fake_deps(self.deps, self.deps1)


    def test_deps_1(self):
        self.assertEquals(self.deps1["a"], self.dt.get_deps_1("a"))
        self.assertEquals(self.deps1["b"], self.dt.get_deps_1("b"))
        self.assertEquals(self.deps1["d"], self.dt.get_deps_1("d"))

    def test_deps(self):
        self.assertEquals(self.deps["a"], self.dt.get_deps("a"))
        self.assertEquals(self.deps["b"], self.dt.get_deps("b"))
        self.assertEquals(self.deps["d"], self.dt.get_deps("d"))

    def test_not_package(self):
        self.assertEquals([], self.dt.get_deps("This is not a valid package name"))
        self.assertEquals([], self.dt.get_deps_1("This is not a valid package name"))
        

class TestBuildQueue(unittest.TestCase):

    def setUp(self):
        deps = {}
        deps1 = {}
        deps1["a"] = ["b"]
        deps["a"] = ["b", "c", "d", "e", "f"]
        deps1["b"] = ["c"]
        deps["b"] = ["c", "d", "e", "f"]
        deps1["c"] = ["d"]
        deps["c"] = ["d", "e", "f"]
        deps1["d"] = ["e"]
        deps["d"] = ["e", "f"]
        deps["e"] = ["f"]
        deps1["e"] = ["f"]
        deps["f"] = []
        deps1["f"] = []

        self.serial_tracker = parallel_build.DependencyTracker()
        self.serial_tracker.load_fake_deps(deps, deps1)

        deps = {}
        deps1 = {}
        deps["a"] = ["b", "c", "d", "e", "f"]
        deps1["a"] = ["b", "c", "d", "e", "f"]
        deps["b"] = []
        deps1["b"] = []
        deps["c"] = []
        deps1["c"] = []
        deps["d"] = []
        deps1["d"] = []
        deps["e"] = []
        deps1["e"] = []
        deps["f"] = []
        deps1["f"] = []
        
        self.parallel_tracker = parallel_build.DependencyTracker()
        self.parallel_tracker.load_fake_deps(deps, deps1)

    # full queue
    def test_full_build(self):
        bq = parallel_build.BuildQueue(["a", "b", "c", "d", "e", "f"], self.serial_tracker)
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())
        
        self.assertEqual("f", bq.get_valid_package())
        self.assertEqual(0, len(bq.built))
        bq.return_built("f")
        self.assertEqual(1, len(bq.built))
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())

        self.assertEqual("e", bq.get_valid_package())
        bq.return_built("e")
        self.assertEqual(2, len(bq.built))
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())

        self.assertEqual("d", bq.get_valid_package())
        bq.return_built("d")
        self.assertEqual(3, len(bq.built))
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())

        self.assertEqual("c", bq.get_valid_package())
        bq.return_built("c")
        self.assertEqual(4, len(bq.built))
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())

        self.assertEqual("b", bq.get_valid_package())
        bq.return_built("b")
        self.assertEqual(5, len(bq.built))
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())

        self.assertEqual("a", bq.get_valid_package())
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())
        bq.return_built("a")
        self.assertEqual(6, len(bq.built))
        self.assertTrue (bq.is_done())
        self.assertTrue (bq.succeeded())


    # partial build
    def test_partial_build(self):
        bq = parallel_build.BuildQueue(["d", "e", "f"], self.serial_tracker)
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())
        
        self.assertEqual("f", bq.get_valid_package())
        self.assertEqual(0, len(bq.built))
        bq.return_built("f")
        self.assertEqual(1, len(bq.built))
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())

        self.assertEqual("e", bq.get_valid_package())
        bq.return_built("e")
        self.assertEqual(2, len(bq.built))
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())

        self.assertEqual("d", bq.get_valid_package())
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())
        bq.return_built("d")
        self.assertEqual(3, len(bq.built))
        self.assertTrue(bq.is_done())
        self.assertTrue(bq.succeeded())


    # abort early
    def test_abort_early(self):
        bq = parallel_build.BuildQueue(["a", "b", "c", "d", "e", "f"], self.serial_tracker)
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())
        self.assertEqual(0, len(bq.built))
        
        self.assertEqual("f", bq.get_valid_package())
        bq.return_built("f")
        self.assertEqual(1, len(bq.built))
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())

        self.assertEqual("e", bq.get_valid_package())
        bq.return_built("e")
        self.assertEqual(2, len(bq.built))
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())

        self.assertEqual("d", bq.get_valid_package())
        bq.return_built("d")
        self.assertEqual(3, len(bq.built))
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())

        bq.stop()
        self.assertTrue(bq.is_done())
        self.assertFalse(bq.succeeded())

        self.assertEqual(None, bq.get_valid_package())

    # many parallel
    def test_parallel_build(self):
        bq = parallel_build.BuildQueue(["a", "b", "c", "d", "e", "f"], self.parallel_tracker)
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())
        
        dependents = ["b", "c", "d", "e", "f"]
        count = 0
        total = 6
        while len(dependents) > 0:
            result= bq.get_valid_package()
            done = len(bq.built)
            pkgs = bq._total_pkgs
            self.assertTrue(result in dependents)
            #print result, done, pkgs
            dependents.remove(result)
            self.assertEqual(count, done)
            self.assertEqual(total, pkgs)
            self.assertFalse(bq.is_done())
            self.assertFalse(bq.succeeded())
            bq.return_built(result)
            count = count + 1
            self.assertFalse(bq.is_done())
            self.assertFalse(bq.succeeded())


        self.assertEqual("a", bq.get_valid_package())
        self.assertFalse(bq.is_done())
        self.assertFalse(bq.succeeded())
        bq.return_built("a")
        self.assertTrue (bq.is_done())
        self.assertTrue (bq.succeeded())


    # stalled(future)
